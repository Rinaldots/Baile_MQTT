#!/usr/bin/env python3
"""
Servidor SLAM: MASt3R + YOLO.

Fluxo:
  1. MAPPING  — pessoa caminha com celular (~10s), coleta keyframes
  2. PROCESSING — MASt3R reconstroi cena, extrai K + mapa 3D de landmarks
  3. TRACKING — cada frame: ORB+PnP → pose camera → YOLO → robo em (x,y,z)

Uso:
    cd yolo_detection
    venv/Scripts/python ../test/mast3r_slam.py

Topicos MQTT publicados:
    observer/camX      — deteccoes brutas
    robots/positions   — posicoes 3D de todos os robos
"""
import asyncio, base64, json, io, ssl, socket, pathlib, time, pickle, sys
import numpy as np
import cv2
from PIL import Image
import paho.mqtt.client as mqtt
from ultralytics import YOLO

DIR         = pathlib.Path(__file__).parent
VENV_SITE   = DIR.parent / 'yolo_detection' / 'venv' / 'Lib' / 'site-packages'
MAST3R_DIR  = DIR.parent / 'yolo_detection' / 'mast3r'
WEIGHTS_DIR = DIR.parent / 'yolo_detection' / 'mast3r_weights'
MODEL_PT    = DIR.parent / 'yolo_detection' / 'runs' / 'robot_detector' / 'weights' / 'best.pt'
MAP_FILE    = DIR / 'slam_map.pkl'
CERT        = DIR / 'cert.pem'
KEY         = DIR / 'key.pem'
WS_PORT     = 8765
CONF_THRESH = 0.5
N_KEYFRAMES = 30    # frames coletados no mapping
ORB_FEATS   = 3000

# Adiciona mast3r ao path
for p in [str(MAST3R_DIR), str(MAST3R_DIR / 'dust3r')]:
    if p not in sys.path:
        sys.path.insert(0, p)

# ── YOLO ─────────────────────────────────────────────────────────────────────
print('Carregando YOLO...')
yolo = YOLO(str(MODEL_PT))
yolo.fuse()
yolo(np.zeros((320, 320, 3), dtype=np.uint8), verbose=False)
print('YOLO pronto.')

# ── ORB + matcher ─────────────────────────────────────────────────────────────
orb     = cv2.ORB_create(ORB_FEATS)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

# ── MQTT ──────────────────────────────────────────────────────────────────────
mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
try:
    mq.connect('127.0.0.1', 1883)
    mq.loop_start()
except Exception as e:
    print(f'MQTT indisponivel: {e}')

# ── Estado global ─────────────────────────────────────────────────────────────
class State:
    IDLE       = 'idle'
    MAPPING    = 'mapping'
    PROCESSING = 'processing'
    TRACKING   = 'tracking'

state           = State.IDLE
mapping_frames  = []   # [(PIL.Image, timestamp)]
K: np.ndarray   = None  # intrinsicos 3x3
dist: np.ndarray = None
lm_pts: np.ndarray  = None  # Nx3 landmarks no mundo
lm_desc: np.ndarray = None  # NxD descritores ORB
current_pose        = None  # (R 3x3, t 3)


# ── MASt3R — roda em executor (bloqueante) ────────────────────────────────────
def run_mast3r(frames_pil: list) -> tuple:
    """
    Recebe lista de PIL.Image, retorna (K, dist, lm_pts, lm_desc).
    """
    import tempfile, os, torch
    from mast3r.model import AsymmetricMASt3R
    from dust3r.inference import inference
    from dust3r.utils.image import load_images
    from dust3r.image_pairs import make_pairs
    from dust3r.cloud_opt import global_aligner, GlobalAlignerMode

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    weights = str(next(WEIGHTS_DIR.glob('*.pth')))
    print(f'MASt3R: carregando pesos de {weights} em {device}')

    mast3r_model = AsymmetricMASt3R.from_pretrained(weights).to(device).eval()

    # Salva frames em disco temporario
    tmpdir = tempfile.mkdtemp()
    paths  = []
    for i, img in enumerate(frames_pil):
        p = os.path.join(tmpdir, f'{i:04d}.jpg')
        img.save(p, quality=90)
        paths.append(p)

    # Inferencia
    images = load_images(paths, size=512)
    pairs  = make_pairs(images, scene_graph='complete', prefilter=None, symmetrize=True)
    output = inference(pairs, mast3r_model, device, batch_size=1, verbose=False)

    # Alinhamento global
    scene = global_aligner(output, device=device,
                           mode=GlobalAlignerMode.PointCloudOptimizer)
    scene.compute_global_alignment(init='mst', niter=300,
                                   schedule='linear', lr=0.01)

    # Intrinsicos (usa media das focais estimadas)
    focals  = scene.get_focals().detach().cpu().numpy()
    focal   = float(focals.mean())
    h, w    = frames_pil[0].height, frames_pil[0].width
    K_mat   = np.array([[focal, 0, w / 2],
                        [0, focal, h / 2],
                        [0,     0,     1]], dtype=np.float64)
    dist_v  = np.zeros(5, dtype=np.float64)

    # Nuvem de pontos 3D com confianca alta
    pts3d_list = scene.get_pts3d()
    conf_list  = scene.get_conf()

    all_pts, all_desc = [], []
    for i, (pts, conf) in enumerate(zip(pts3d_list, conf_list)):
        pts_np  = pts.detach().cpu().numpy()   # H×W×3
        conf_np = conf.detach().cpu().numpy()  # H×W

        img_cv  = cv2.imread(paths[i])
        h_i, w_i = img_cv.shape[:2]
        kps, descs = orb.detectAndCompute(img_cv, None)
        if descs is None:
            continue

        pts_h, pts_w = pts_np.shape[:2]
        for kp, desc in zip(kps, descs):
            px = int(kp.pt[0] / w_i * pts_w)
            py = int(kp.pt[1] / h_i * pts_h)
            px, py = min(px, pts_w - 1), min(py, pts_h - 1)
            if conf_np[py, px] > 1.5:          # so pontos confiaveis
                all_pts.append(pts_np[py, px])
                all_desc.append(desc)

    lm_pts_out  = np.array(all_pts,  dtype=np.float32)
    lm_desc_out = np.array(all_desc, dtype=np.uint8)

    print(f'MASt3R: {len(lm_pts_out)} landmarks, K={np.diag(K_mat)[:2]}')
    return K_mat, dist_v, lm_pts_out, lm_desc_out


def run_mast3r_safe(frames_pil: list) -> tuple:
    """Wrapper que converte StopIteration em RuntimeError (asyncio nao aceita StopIteration em Futures)."""
    try:
        return run_mast3r(frames_pil)
    except StopIteration as e:
        raise RuntimeError(f'MASt3R StopIteration interna: {e}') from None


# ── Estimacao de pose (ORB + PnP) ─────────────────────────────────────────────
def estimate_pose(bgr: np.ndarray):
    global current_pose, lm_pts, lm_desc, K, dist

    kps, descs = orb.detectAndCompute(bgr, None)
    if descs is None or len(kps) < 12:
        return None

    matches = matcher.knnMatch(descs, lm_desc, k=2)
    good = [m for m, n in matches if m.distance < 0.75 * n.distance]
    if len(good) < 12:
        return None

    pts2d = np.float32([kps[m.queryIdx].pt      for m in good])
    pts3d = np.float32([lm_pts[m.trainIdx]       for m in good])

    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        pts3d, pts2d, K, dist,
        iterationsCount=200, reprojectionError=4.0,
        confidence=0.99, flags=cv2.SOLVEPNP_SQPNP,
    )
    if not ok or inliers is None or len(inliers) < 8:
        return None

    R, _ = cv2.Rodrigues(rvec)
    current_pose = (R, tvec.flatten())
    return current_pose


# ── Back-projection pixel → plano z=0 ────────────────────────────────────────
def backproject_floor(K, R, t, u, v):
    C   = (-R.T @ t)                            # centro da camera no mundo
    ray = R.T @ np.linalg.inv(K) @ [u, v, 1.0] # direcao do raio
    ray /= np.linalg.norm(ray)
    if abs(ray[2]) < 1e-6:
        return None
    lam = -C[2] / ray[2]
    if lam < 0:
        return None
    p = C + lam * ray
    return round(float(p[0]), 1), round(float(p[1]), 1), 0.0


# ── WebSocket handler ─────────────────────────────────────────────────────────
async def handle(ws):
    global state, mapping_frames, K, dist, lm_pts, lm_desc

    print(f'[+] {ws.remote_address}')
    await ws.send(json.dumps({'status': state}))

    try:
        async for raw in ws:
            data = json.loads(raw)
            cmd  = data.get('cmd')

            # ── Comandos de controle ──────────────────────────────────────
            if cmd == 'start_mapping':
                state, mapping_frames = State.MAPPING, []
                print('>> MAPPING iniciado — caminhe pelo ambiente')
                await ws.send(json.dumps({'status': 'mapping', 'frames': 0}))
                continue

            if cmd == 'stop_mapping':
                if len(mapping_frames) < 6:
                    await ws.send(json.dumps({'status': 'error',
                                              'msg': 'Poucos frames (min 6)'}))
                    continue

                state = State.PROCESSING
                await ws.send(json.dumps({'status': 'processing',
                                          'frames': len(mapping_frames)}))
                print(f'>> MASt3R rodando em {len(mapping_frames)} frames...')

                loop = asyncio.get_event_loop()
                try:
                    K, dist, lm_pts, lm_desc = await loop.run_in_executor(
                        None, run_mast3r_safe, mapping_frames
                    )
                    with open(MAP_FILE, 'wb') as f:
                        pickle.dump({'K': K, 'dist': dist,
                                     'lm_pts': lm_pts, 'lm_desc': lm_desc}, f)
                    state = State.TRACKING
                    print(f'>> TRACKING ativo ({len(lm_pts)} landmarks)')
                    await ws.send(json.dumps({'status': 'tracking',
                                              'landmarks': len(lm_pts)}))
                except Exception as e:
                    state = State.IDLE
                    await ws.send(json.dumps({'status': 'error', 'msg': str(e)}))
                continue

            if cmd == 'reset':
                state, mapping_frames = State.IDLE, []
                K = dist = lm_pts = lm_desc = None
                MAP_FILE.unlink(missing_ok=True)
                await ws.send(json.dumps({'status': 'idle'}))
                continue

            # ── Frame de camera ───────────────────────────────────────────
            if 'img' not in data:
                continue

            cam_id  = data['cam']
            img_pil = Image.open(io.BytesIO(base64.b64decode(data['img'])))

            if state == State.MAPPING:
                # Amostra keyframes (max N_KEYFRAMES)
                if len(mapping_frames) < N_KEYFRAMES:
                    mapping_frames.append(img_pil)
                await ws.send(json.dumps({'status': 'mapping',
                                          'frames': len(mapping_frames),
                                          'target': N_KEYFRAMES}))
                continue

            if state != State.TRACKING:
                continue

            t0 = time.perf_counter()

            # Pose da camera
            bgr  = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
            pose = estimate_pose(bgr)

            # Deteccao YOLO
            results = yolo(img_pil, verbose=False)[0]
            dets = []
            for box in results.boxes:
                if float(box.conf[0]) < CONF_THRESH:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                det = {
                    'pixel'    : {'u': cx, 'v': cy},
                    'score'    : float(box.conf[0]),
                    'bbox_send': [x1, y1, x2, y2],
                    'send_size': list(img_pil.size),
                    'color'    : 'robot',
                }
                if pose is not None:
                    R, t = pose
                    world = backproject_floor(K, R, t, cx, cy)
                    if world:
                        det['world'] = {'x': world[0], 'y': world[1], 'z': world[2]}
                dets.append(det)

            payload = json.dumps({'cam': cam_id,
                                   't': int(time.time() * 1000),
                                   'detections': dets})
            mq.publish(f'observer/cam{cam_id}', payload)

            world_dets = [d for d in dets if 'world' in d]
            if world_dets:
                mq.publish('robots/positions', json.dumps({
                    'cam': cam_id, 't': int(time.time() * 1000),
                    'robots': [{'id': i, **d['world'], 'score': d['score']}
                               for i, d in enumerate(world_dets)],
                }))

            await ws.send(payload)
            ms = (time.perf_counter() - t0) * 1000
            flag = f'{len(dets)}det  pose={"ok" if pose else "x"}  {ms:.0f}ms'
            print(f'cam{cam_id}: {flag}')

    except Exception as e:
        print(f'[-] {ws.remote_address}: {e}')


# ── Main ──────────────────────────────────────────────────────────────────────
def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    finally:
        s.close()


async def main():
    global state, K, dist, lm_pts, lm_desc

    if MAP_FILE.exists():
        with open(MAP_FILE, 'rb') as f:
            m = pickle.load(f)
        K, dist, lm_pts, lm_desc = m['K'], m['dist'], m['lm_pts'], m['lm_desc']
        state = State.TRACKING
        print(f'Mapa carregado: {len(lm_pts)} landmarks')
    else:
        print('Sem mapa. No celular: pressione "Iniciar Mapeamento".')

    import websockets
    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(str(CERT), str(KEY))

    ip = local_ip()
    print(f'\nSLAM server: wss://{ip}:{WS_PORT}\n')

    async with websockets.serve(handle, '0.0.0.0', WS_PORT, ssl=ssl_ctx):
        await asyncio.Future()


if __name__ == '__main__':
    asyncio.run(main())
