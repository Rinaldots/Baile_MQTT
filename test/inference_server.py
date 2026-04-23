#!/usr/bin/env python3
"""
Servidor de inferencia YOLO.
Recebe frames JPEG do celular via WSS, roda YOLO na GPU,
aplica homografia -> coordenadas mundo (x, y, z=0) e publica no MQTT.

Uso:
    cd yolo_detection
    venv/Scripts/python ../test/inference_server.py
"""
import asyncio, base64, json, io, ssl, socket, pathlib, time, threading
import numpy as np
import paho.mqtt.client as mqtt
from PIL import Image
from ultralytics import YOLO

DIR       = pathlib.Path(__file__).parent
CERT      = DIR / 'cert.pem'
KEY       = DIR / 'key.pem'
WS_PORT   = 8765
MQTT_HOST = '127.0.0.1'
MQTT_PORT = 1883
MODEL_PT  = DIR.parent / 'yolo_detection' / 'runs' / 'robot_detector' / 'weights' / 'best.pt'
CONF      = 0.5

# --- Modelo -----------------------------------------------------------------
print('Carregando modelo YOLO...')
model = YOLO(str(MODEL_PT))
model.fuse()
model(np.zeros((320, 320, 3), dtype=np.uint8), verbose=False)  # aquece GPU
print('Modelo pronto.')

# --- Estado por camera ------------------------------------------------------
# { cam_id: np.ndarray 3x3 }  — homografia pixel(full-res) -> mundo (cm)
H_store: dict[str, np.ndarray] = {}
H_lock = threading.Lock()

# { cam_id: (video_w, video_h) }
video_size: dict[str, tuple] = {}

# --- MQTT -------------------------------------------------------------------
mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

def on_mqtt_message(client, userdata, msg):
    """Armazena homografia assim que a calibracao chega."""
    # topico: cam/calibration/<cam_id>
    parts = msg.topic.split('/')
    if len(parts) != 3:
        return
    cam_id = parts[2]
    try:
        d = json.loads(msg.payload)
        if d.get('H') and len(d['H']) == 9:
            H = np.array(d['H'], dtype=np.float64).reshape(3, 3)
            with H_lock:
                H_store[cam_id] = H
            print(f'[cal] cam{cam_id}: H atualizada (RMSE={d.get("rmse","?"):.1f} cm)')
    except Exception as e:
        print(f'[cal] erro parse: {e}')

mq.on_message = on_mqtt_message

try:
    mq.connect(MQTT_HOST, MQTT_PORT)
    mq.subscribe('cam/calibration/+')
    mq.loop_start()
    print(f'MQTT conectado em {MQTT_HOST}:{MQTT_PORT}')
except Exception as e:
    print(f'MQTT indisponivel ({e})')

# --- Homografia --------------------------------------------------------------
def apply_H(H: np.ndarray, u: float, v: float) -> tuple[float, float]:
    pt  = H @ np.array([u, v, 1.0])
    return float(pt[0] / pt[2]), float(pt[1] / pt[2])

# --- WebSocket handler -------------------------------------------------------
async def handle(ws):
    addr = ws.remote_address
    print(f'[+] cam conectada: {addr}')
    try:
        async for msg in ws:
            t0   = time.perf_counter()
            data = json.loads(msg)

            cam_id  = data['cam']
            img     = Image.open(io.BytesIO(base64.b64decode(data['img'])))
            send_w, send_h = img.size

            # Resolucao original do video (necessaria para re-escalar para o H calibrado)
            vid_w = data.get('video_w', send_w)
            vid_h = data.get('video_h', send_h)
            video_size[cam_id] = (vid_w, vid_h)

            sx = vid_w / send_w   # escala send -> full-res
            sy = vid_h / send_h

            results = model(img, verbose=False)[0]

            with H_lock:
                H = H_store.get(cam_id)

            dets = []
            for box in results.boxes:
                if float(box.conf[0]) < CONF:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                # centro em coordenadas do frame enviado (640x360)
                cx_s = (x1 + x2) / 2
                cy_s = (y1 + y2) / 2

                # reescala para resolucao original (1280x720) — mesma escala do H calibrado
                cx = cx_s * sx
                cy = cy_s * sy

                det = {
                    'pixel'    : {'u': cx, 'v': cy},
                    'bbox_full': [x1*sx, y1*sy, x2*sx, y2*sy],
                    'bbox_send': [x1, y1, x2, y2],
                    'send_size': [send_w, send_h],
                    'area'     : (x2 - x1) * (y2 - y1) * sx * sy,
                    'color'    : 'robot',
                    'score'    : float(box.conf[0]),
                }

                if H is not None:
                    wx, wy = apply_H(H, cx, cy)
                    det['world'] = {'x': round(wx, 1), 'y': round(wy, 1), 'z': 0.0}

                dets.append(det)

            payload = json.dumps({
                'cam'       : cam_id,
                't'         : int(time.time() * 1000),
                'detections': dets,
            })

            # Publica observacoes para o bundle_adjustment.py
            mq.publish(f'observer/cam{cam_id}', payload)

            # Publica posicoes mundo se calibrado
            world_dets = [d for d in dets if 'world' in d]
            if world_dets:
                robots = [{'id': i, **d['world'], 'score': d['score']}
                          for i, d in enumerate(world_dets)]
                mq.publish('robots/positions', json.dumps({
                    'cam': cam_id,
                    't'  : int(time.time() * 1000),
                    'robots': robots,
                }))

            # Devolve resultado para o celular exibir
            await ws.send(payload)

            ms = (time.perf_counter() - t0) * 1000
            cal = 'cal' if H is not None else 'sem_cal'
            print(f'cam{cam_id}: {len(dets)} robos  {ms:.1f}ms  [{cal}]')

    except Exception as e:
        print(f'[-] cam desconectada {addr}: {e}')

# --- Main -------------------------------------------------------------------
def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    finally:
        s.close()

async def main():
    import websockets
    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(str(CERT), str(KEY))

    ip = local_ip()
    print(f'\nServidor WSS: wss://{ip}:{WS_PORT}')
    print('Topicos MQTT publicados:')
    print('  observer/camX     — deteccoes brutas (para bundle_adjustment.py)')
    print('  robots/positions  — posicoes mundo (x,y,z) de todos os robos\n')

    async with websockets.serve(handle, '0.0.0.0', WS_PORT, ssl=ssl_ctx):
        await asyncio.Future()

if __name__ == '__main__':
    asyncio.run(main())
