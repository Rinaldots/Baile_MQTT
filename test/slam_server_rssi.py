import json, time, threading, logging, argparse
import numpy as np
import paho.mqtt.client as mqtt

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("SLAM")

# ─────────────────────────────────────────────
# CONFIGURAÇÕES RSSI CLÁSSICO
# ─────────────────────────────────────────────
MQTT_BROKER        = "10.0.0.134"
MQTT_PORT          = 1883
ROBOT_FILTER       = "#"

# Constantes do Path Loss Model (Calibre para seu ambiente!)
RSSI_AT_1M         = -45.0  # Força do sinal a exatos 1 metro do roteador (meça o seu no ESP32)
PATH_LOSS_EXPONENT = 2.5    # Varia de 2.0 (aberto) a 4.0 (muitos obstáculos/paredes)

EKF_SIGMA_VX       = 0.01 
EKF_SIGMA_VY       = 0.01
EKF_SIGMA_OMEGA    = 0.01
EKF_SIGMA_RSSI     = 4.0    # Incerteza base da leitura do RSSI (metros)

MAX_ODOM_SPEED_MS  = 2.0
MAX_ODOM_OMEGA_RS  = 6.0
MAX_ODOM_DIST_M    = 0.5

class RobotEKF:
    def __init__(self):
        self.x = np.zeros(3)
        self.P = np.diag([1.0, 1.0, np.pi])
        self.initialized = False
        self.lock = threading.Lock()

    def predict(self, vx, vy, omega, dt):
        if dt <= 0 or dt > 2.0: return
        with self.lock:
            th = self.x[2]
            c, s = np.cos(th), np.sin(th)
            F = np.eye(3)
            F[0, 2] = (-vx * s - vy * c) * dt
            F[1, 2] = ( vx * c - vy * s) * dt

            self.x[0] += (vx * c - vy * s) * dt
            self.x[1] += (vx * s + vy * c) * dt
            self.x[2]  = (self.x[2] + omega * dt + np.pi) % (2 * np.pi) - np.pi

            Q = np.diag([(EKF_SIGMA_VX * dt)**2, (EKF_SIGMA_VY * dt)**2, (EKF_SIGMA_OMEGA * dt)**2])
            self.P = F @ self.P @ F.T + Q
            self.initialized = True

    def update_theta(self, th_odom, sigma=0.05):
        """O ângulo baseia-se 100% na odometria real."""
        with self.lock:
            H = np.array([[0.0, 0.0, 1.0]])
            inn = (th_odom - self.x[2] + np.pi) % (2 * np.pi) - np.pi
            S = float((H @ self.P @ H.T + np.array([[sigma**2]]))[0, 0])
            K = (self.P @ H.T / S).flatten()
            self.x = self.x + K * inn
            self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi
            self.P = (np.eye(3) - np.outer(K, H)) @ self.P
            self.initialized = True

    def update_distance(self, dist, anchor_x, anchor_y, sigma):
        """Atualização de Trilateração (Range-Only) usando APs Wi-Fi."""
        with self.lock:
            if not self.initialized: return
            
            dx = self.x[0] - anchor_x
            dy = self.x[1] - anchor_y
            d_hat = np.hypot(dx, dy)
            if d_hat < 0.1: d_hat = 0.1
            
            # Jacobiana H foca apenas na alteração de X e Y em favor da distância
            H = np.array([[dx / d_hat, dy / d_hat, 0.0]])
            y_inn = np.array([dist - d_hat])
            R = np.array([[sigma**2]])
            
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)
            
            self.x = self.x + (K @ y_inn).flatten()
            self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi
            self.P = (np.eye(3) - K @ H) @ self.P

    @property
    def pose(self):
        with self.lock: return self.x.copy(), self.P.copy()

class RobotState:
    def __init__(self, robot_id):
        self.id = robot_id
        self.ekf = RobotEKF()
        self.last_rssi = -70
        self.last_mac = robot_id
        self.vx_odom = 0.0
        self.vy_odom = 0.0
        self.w_odom = 0.0
        self.last_odom_state = None
        self.last_publish_ms = 0
        self.lock = threading.Lock()

class SLAMServer:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, robot=ROBOT_FILTER):
        self.broker = broker
        self.port = port
        self.robot = robot
        self.robots = {}
        self.lock = threading.Lock()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def _on_connect(self, client, *_):
        prefix = f"/{self.robot}" if self.robot != "#" else ""
        client.subscribe(f"{prefix}/+/CSI" if self.robot == "#" else f"/{self.robot}/CSI")
        client.subscribe(f"{prefix}/+/odom" if self.robot == "#" else f"/{self.robot}/odom")
        log.info(f"MQTT conectado. Servidor de Posicionamento RSSI Clássico Iniciado.")

    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            parts = topic.strip("/").split("/")
            if len(parts) < 2: return
            mac, tipo = parts[0], parts[-1]
            payload = json.loads(msg.payload.decode())
            
            with self.lock:
                if mac not in self.robots:
                    self.robots[mac] = RobotState(mac)
                robot = self.robots[mac]

            if tipo == "CSI":
                self._handle_rssi(robot, mac, payload)
            elif tipo == "odom":
                self._handle_odom(robot, payload)
        except Exception as e:
            pass

    def _handle_rssi(self, robot, mac, payload):
        rssi = payload.get("rssi", -70)
        if rssi == 0: return
        
        # 1. Equação Clássica: Converte RSSI para Distância em metros
        # d = 10 ^ ((A - RSSI) / (10 * n))
        dist_m = 10.0 ** ((RSSI_AT_1M - rssi) / (10.0 * PATH_LOSS_EXPONENT))
        
        robot.last_rssi = rssi
        robot.last_mac = mac

        # 2. Ignora leituras instáveis se o robô estiver parado
        is_stopped = abs(robot.vx_odom) < 0.02 and abs(robot.w_odom) < 0.02
        if is_stopped: return

        # 3. Modelagem de Erro: Incerteza do RSSI cresce linearmente com a distância real
        sigma = EKF_SIGMA_RSSI * (1.0 + (dist_m * 0.2))

        # 4. Range-Only EKF Update
        # Assumindo que seu roteador de Wi-Fi principal (Onde o ESP32 está conectado) fica no eixo (0,0) absoluto.
        # Se você tivesse mais roteadores no payload, faria um "for anchor in payload_rssi_list" chamando update_distance().
        robot.ekf.update_distance(dist_m, anchor_x=0.0, anchor_y=0.0, sigma=sigma)
        
        self._publish(robot, dist_m=dist_m)

    def _handle_odom(self, robot, payload):
        x_m = float(payload.get("x",0))
        y_m = float(payload.get("y",0))
        th_m = float(payload.get("theta",0))
        t_ms = float(payload.get("t",0))

        if robot.last_odom_state is None:
            robot.last_odom_state = (x_m, y_m, th_m, t_ms)
            return

        last_x, last_y, last_th, last_t = robot.last_odom_state
        robot.last_odom_state = (x_m, y_m, th_m, t_ms)

        dt = np.clip((t_ms - last_t) / 1000.0, 0.001, 1.0)
        dx, dy = x_m - last_x, y_m - last_y
        dth = (th_m - last_th + np.pi) % (2 * np.pi) - np.pi

        if np.hypot(dx, dy)/dt > MAX_ODOM_SPEED_MS or abs(dth/dt) > MAX_ODOM_OMEGA_RS:
            robot.ekf.update_theta(th_m, sigma=0.05)
            return

        c, s = np.cos(-last_th), np.sin(-last_th)
        robot.vx_odom = (dx * c - dy * s) / dt
        robot.vy_odom = (dx * s + dy * c) / dt
        robot.w_odom = dth / dt

        robot.ekf.predict(robot.vx_odom, robot.vy_odom, robot.w_odom, dt)
        robot.ekf.update_theta(th_m, sigma=0.05)

        x, P = robot.ekf.pose
        diff_theta = (x[2] - th_m + np.pi) % (2 * np.pi) - np.pi
        log.info(f"[{robot.id[:11]}] RSSI={robot.last_rssi}dBm | Pos=({x[0]:.2f}, {x[1]:.2f})m | Distância Estimada: {robot.ekf.x[0]:.1f}m | T-Diff={np.degrees(diff_theta):.1f}°")
        self._publish(robot)

    def _publish(self, robot, dist_m=0.0):
        now_ms = int(time.time() * 1000)
        if now_ms - robot.last_publish_ms < 50: return
        robot.last_publish_ms = now_ms

        x, P = robot.ekf.pose
        msg = {
            "mac": robot.last_mac, "rssi": robot.last_rssi,
            "pos": {"x": round(float(x[0]), 4), "y": round(float(x[1]), 4), "theta": round(float(x[2]), 4)},
            "cov": {"sx": round(float(np.sqrt(max(P[0,0],0))),4), "sy": round(float(np.sqrt(max(P[1,1],0))),4), "st": round(float(np.sqrt(max(P[2,2],0))),4)},
            "tofs_ns": [round(dist_m, 2)], # Re-usado para injetar a distância absoluta direto no log
            "map_ready": True, "t": now_ms
        }
        self.client.publish(f"/{robot.id}/slam", json.dumps(msg), qos=0)

    def run(self):
        log.info("="*55)
        log.info(" Iniciando Range-Only RSSI Clássico")
        log.info(" Sem Neural Net, Sem Matrizes Complexas")
        log.info("="*55)
        self.client.connect(self.broker, self.port)
        self.client.loop_forever()

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--broker", default=MQTT_BROKER)
    p.add_argument("--port", default=MQTT_PORT, type=int)
    p.add_argument("--robot", default=ROBOT_FILTER)
    p.add_argument("--debug", action="store_true")
    args = p.parse_args()
    if args.debug: logging.getLogger().setLevel(logging.DEBUG)
    SLAMServer(broker=args.broker, port=args.port, robot=args.robot).run()