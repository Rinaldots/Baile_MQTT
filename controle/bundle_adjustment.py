"""
Calibracao automatica multi-camera por Bundle Adjustment.

Usa odometria do ESP32 (topico MQTT 'telemetry') como referencia de
mundo, junto com observacoes de pixel dos LEDs vindas de cada celular
rodando observer.html (topico 'observer/camX').

Sem marcadores fisicos nem cliques manuais: basta mover o robo pelo
ambiente por alguns minutos e o script publica as homografias
pixel -> cm (plano do chao) em cam/calibration/camX (retained).
"""
import json
import time
import threading
from collections import defaultdict

import numpy as np
from scipy.optimize import least_squares
import paho.mqtt.client as mqtt

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# --- Configuracao --------------------------------------------------------
BROKER          = "10.0.0.134"
PORT            = 1883
SYNC_WINDOW_MS  = 80     # janela de sincronizacao entre cameras
MIN_FRAMES      = 60     # frames com 2+ cameras antes de rodar BA
MIN_CAMS        = 2
BA_INTERVAL_S   = 30     # reotimiza a cada N segundos com dados novos


# --- Coleta de dados -----------------------------------------------------

class DataCollector:
    def __init__(self):
        self.cam_obs = defaultdict(list)  # cam_id -> [(t_ms, u, v)]
        self.odom    = []                 # [(t_ms, x_cm, y_cm)]
        self.lock    = threading.Lock()

    def add_obs(self, cam_id, t, u, v):
        with self.lock:
            self.cam_obs[cam_id].append((t, float(u), float(v)))

    def add_odom(self, t, x_m, y_m):
        with self.lock:
            self.odom.append((t, x_m * 100.0, y_m * 100.0))  # m -> cm

    def cam_ids(self):
        with self.lock:
            return set(self.cam_obs.keys())

    def get_odom_at(self, t):
        with self.lock:
            odom = list(self.odom)
        if len(odom) < 2:
            return None
        lo, hi = 0, len(odom) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if odom[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        if lo == 0:
            return odom[0][1], odom[0][2]
        t0, x0, y0 = odom[lo - 1]
        t1, x1, y1 = odom[lo]
        a = (t - t0) / (t1 - t0) if t1 != t0 else 0.0
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0)

    def synchronize(self):
        """
        Agrupa observacoes dentro de SYNC_WINDOW_MS.
        Mantem so frames onde 2+ cameras viram o LED.
        Retorna: [(t_ms, (odom_x, odom_y), {cam_id: (u, v)})]
        """
        with self.lock:
            all_obs = []
            for cam_id, obs_list in self.cam_obs.items():
                for t, u, v in obs_list:
                    all_obs.append((t, cam_id, u, v))

        all_obs.sort()
        frames, i = [], 0

        while i < len(all_obs):
            t0 = all_obs[i][0]
            group = {}
            while i < len(all_obs) and all_obs[i][0] <= t0 + SYNC_WINDOW_MS:
                t, cam_id, u, v = all_obs[i]
                if cam_id not in group:
                    group[cam_id] = (u, v)
                i += 1

            if len(group) >= 2:
                t_mid = t0 + SYNC_WINDOW_MS / 2
                odom  = self.get_odom_at(t_mid)
                if odom:
                    frames.append((t_mid, odom, group))

        return frames


# --- Inicializacao das homografias ---------------------------------------

def _dlt_homography(src, dst):
    """DLT: H tal que dst ~ H * src (pixel -> mundo)."""
    A = []
    for (u, v), (X, Y) in zip(src, dst):
        A.append([-u, -v, -1,  0,  0,  0, u * X, v * X, X])
        A.append([ 0,  0,  0, -u, -v, -1, u * Y, v * Y, Y])
    A = np.array(A)
    _, _, Vt = np.linalg.svd(A)
    h = Vt[-1]
    return (h / h[-1]).reshape(3, 3)


def initialize_homographies(frames, cam_ids):
    """
    Para cada camera, usa os frames onde ela aparece + odometria como
    posicao de mundo -> findHomography/DLT. Resultado e chute inicial
    para o BA.
    """
    H_flat = []
    for cam_id in sorted(cam_ids):
        src, dst = [], []
        for _, (ox, oy), cam_obs in frames:
            if cam_id in cam_obs:
                u, v = cam_obs[cam_id]
                src.append([u, v])
                dst.append([ox, oy])

        src = np.array(src, np.float64)
        dst = np.array(dst, np.float64)

        if len(src) >= 4:
            if HAS_CV2:
                H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 15.0)
                if H is None:
                    H, _ = cv2.findHomography(src, dst)
            else:
                H = _dlt_homography(src, dst)
            H = H / H[2, 2]
        else:
            # Chute grosseiro
            H = np.array([[0.5, 0.0, -160.0],
                          [0.0, 0.5, -120.0],
                          [0.0, 0.0,    1.0]], np.float64)

        H_flat.extend(H.flatten()[:8].tolist())  # h[8]=1 fixo

    return H_flat


# --- Bundle Adjustment ---------------------------------------------------

def run_ba(frames, cam_ids):
    cam_list = sorted(cam_ids)
    n_cams   = len(cam_list)
    cam_idx  = {c: i for i, c in enumerate(cam_list)}
    n_frames = len(frames)

    print(f"\nBA: {n_cams} cameras, {n_frames} frames")

    H_init     = initialize_homographies(frames, cam_ids)
    world_init = np.array([f[1] for f in frames]).flatten()
    params0    = np.array(H_init + list(world_init), dtype=np.float64)

    # Ancoras: primeiros frames fixam escala e origem
    n_anchors = max(10, n_frames // 8)
    anchors   = [(i, frames[i][1][0], frames[i][1][1]) for i in range(n_anchors)]
    ANCHOR_W  = 8.0

    def residuals(params):
        H_list = []
        for i in range(n_cams):
            h = np.append(params[i * 8:(i + 1) * 8], 1.0)
            H_list.append(h.reshape(3, 3))

        world = params[n_cams * 8:].reshape(-1, 2)
        res   = []

        # Reprojecao pixel -> mundo
        for fi, (_, _, cam_obs) in enumerate(frames):
            Xw, Yw = world[fi]
            for cam_id, (u, v) in cam_obs.items():
                H  = H_list[cam_idx[cam_id]]
                pt = H @ np.array([u, v, 1.0])
                Xp, Yp = pt[0] / pt[2], pt[1] / pt[2]
                res.extend([Xp - Xw, Yp - Yw])

        # Ancoras
        for fi, ox, oy in anchors:
            Xw, Yw = world[fi]
            res.extend([(Xw - ox) * ANCHOR_W, (Yw - oy) * ANCHOR_W])

        return res

    result = least_squares(
        residuals, params0,
        method='trf',
        loss='huber',
        f_scale=8.0,
        max_nfev=3000,
        verbose=1,
    )

    calibrations = {}
    for i, cam_id in enumerate(cam_list):
        h = np.append(result.x[i * 8:(i + 1) * 8], 1.0)
        calibrations[cam_id] = h.reshape(3, 3).tolist()

    rmse = np.sqrt(result.cost * 2 / max(len(result.fun), 1))
    print(f"BA concluido - RMSE ~ {rmse:.1f} cm")
    return calibrations, rmse


# --- Loop principal MQTT -------------------------------------------------

def main():
    collector = DataCollector()

    def on_message(client, userdata, msg):
        try:
            data  = json.loads(msg.payload)
            topic = msg.topic

            if topic.startswith("observer/cam"):
                cam_id = topic.split("/")[-1]
                t      = data.get("t", int(time.time() * 1000))
                # Aceita deteccoes YOLO ("robot") e LEDs ("blue")
                robot_dets = [d for d in data.get("detections", [])
                              if d.get("color") in ("robot", "blue")]
                if robot_dets:
                    # Usa o de maior confianca para calibracao
                    best = max(robot_dets, key=lambda d: d.get("score", 1.0))
                    collector.add_obs(cam_id, t,
                                      best["pixel"]["u"], best["pixel"]["v"])

            elif topic == "telemetry":
                t = int(time.time() * 1000)
                collector.add_odom(t, data.get("x", 0.0), data.get("y", 0.0))

        except Exception as e:
            print(f"Parse error: {e}")

    client = mqtt.Client()
    client.on_message = on_message
    client.connect(BROKER, PORT)
    client.subscribe([("observer/+", 0), ("telemetry", 0)])
    client.loop_start()

    print("Coletando dados - mova o robo pelo ambiente.")
    print(f"Aguardando >={MIN_FRAMES} frames com {MIN_CAMS}+ cameras "
          f"simultaneas...\n")

    last_ba = 0.0

    while True:
        time.sleep(5)
        frames  = collector.synchronize()
        cam_ids = collector.cam_ids()

        print(f"[{time.strftime('%H:%M:%S')}] "
              f"cameras={len(cam_ids)}  "
              f"frames_sync={len(frames)}  "
              f"odom={len(collector.odom)}")

        if (len(frames) >= MIN_FRAMES and
                len(cam_ids) >= MIN_CAMS and
                time.time() - last_ba > BA_INTERVAL_S):
            try:
                calibrations, rmse = run_ba(frames, cam_ids)

                for cam_id, H in calibrations.items():
                    payload = json.dumps({
                        "H":    [v for row in H for v in row],
                        "rmse": rmse,
                        "ts":   int(time.time() * 1000),
                    })
                    client.publish(f"cam/calibration/{cam_id}",
                                   payload, retain=True, qos=1)
                    print(f"  -> cam/{cam_id} publicada (RMSE={rmse:.1f} cm)")

                last_ba = time.time()
            except Exception as e:
                print(f"Erro BA: {e}")


if __name__ == "__main__":
    main()
