"""
Neural RF SLAM Server — EKF Edition
=====================================
CSI (ESP32) + Odometria do swarm → EKF → Pose precisa por robô

Fluxo de dados:
  /{MAC}/CSI     → CSI bruto → ToFs → posição neural (medição EKF)
  /{MAC}/odom    → {vx, vy, omega, dt} → predição EKF
  /{MAC}/slam    ← posição EKF fundida (publish)
  /{MAC}/slam_map← mapa de âncoras virtuais (publish)

Instalação:
    pip install paho-mqtt numpy scipy torch

Uso:
    python slam_server_ekf.py
    python slam_server_ekf.py --broker 192.168.1.1 --robot "#"
"""

import json, time, threading, logging, argparse, random
import numpy as np
from collections import deque
import paho.mqtt.client as mqtt

# ─────────────────────────────────────────────
#  LOGGING
# ─────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("SLAM")

# ─────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────
MQTT_BROKER        = "10.0.0.134"
MQTT_PORT          = 1883
ROBOT_FILTER       = "#"          # "#" para todos os robôs; ou MAC específico

N_VIRTUAL_ANCHORS  = 12
BANDWIDTH_HZ       = 20e6
SPEED_OF_LIGHT     = 3e8

# Buffer de treino
TRAIN_BUFFER_SIZE  = 50
RETRAIN_EVERY      = 20           # re-treina a cada N amostras novas

# EKF — ruídos de processo e medição
EKF_SIGMA_VX       = 0.01         # m/s  — ruído velocidade X
EKF_SIGMA_VY       = 0.01        # m/s  — ruído velocidade Y
EKF_SIGMA_OMEGA    = 0.01         # rad/s — ruído angular
EKF_SIGMA_CSI      = 0.5              # m    — ruído medição CSI (≈30 cm)

# Publicação EKF: mínimo intervalo entre publishes (ms)
EKF_MIN_PUBLISH_MS = 50           # 20 Hz máximo

# Limites físicos para detecção de salto de odometria
MAX_ODOM_SPEED_MS  = 2.0          # m/s  — velocidade linear máxima realista
MAX_ODOM_OMEGA_RS  = 6.0          # rad/s — velocidade angular máxima realista
MAX_ODOM_DIST_M    = 0.5          # m    — deslocamento máximo por pacote (50 cm)

# Qualidade mínima do sinal CSI
MIN_RSSI_DBM       = -80          # dBm  — ignora CSI com sinal muito fraco

# ─────────────────────────────────────────────
#  TORCH (opcional — fallback geométrico sem ele)
# ─────────────────────────────────────────────
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from scipy.optimize import linear_sum_assignment
    TORCH_OK = True
    log.info("PyTorch disponível — Neural RF SLAM ativo")
except ImportError:
    TORCH_OK = False
    log.warning("PyTorch NÃO encontrado — usando estimativa geométrica (fallback)")


# ═════════════════════════════════════════════
#  1. CSI PARSING (Limpeza + Sanitização Linear)
# ═════════════════════════════════════════════
def parse_csi(data_list):
    """Pares (imag, real) → array complexo H com Sanitização CFO/SFO"""
    arr = np.array(data_list, dtype=np.float32)
    n = (len(arr) // 2) * 2
    arr = arr[:n]
    H_raw = arr[1::2] + 1j * arr[0::2]
    valid = list(range(1, 27)) + list(range(38, min(64, len(H_raw))))
    H = H_raw[[i for i in valid if i < len(H_raw)]]

    if len(H) < 4:
        return H

    # --- Sanitização de Fase (Remove CFO, SFO e Delay de Pacote) ---
    # 1. Extrai a fase enrolada
    phases = np.angle(H)
    # 2. Desenrola a fase (+- PI vira contínuo)
    unwrapped = np.unwrap(phases)
    # 3. Fit de uma reta (Regressão Linear) para achar o desvio de relógio
    indices = np.arange(len(unwrapped))
    A = np.vstack([indices, np.ones(len(indices))]).T
    m, c = np.linalg.lstsq(A, unwrapped, rcond=None)[0]
    # 4. Remove a 'inclinação' falsa gerada pela dessincronização de hardware
    sanitized_phases = unwrapped - (m * indices + c)
    
    # 5. Reconstrói o número complexo com a amplitude intocada, mas a fase corrigida
    H_clean = np.abs(H) * np.exp(1j * sanitized_phases)

    return H_clean


# ═════════════════════════════════════════════
#  2. ToF via Algoritmo MUSIC (Super-Resolução)
# ═════════════════════════════════════════════
def extract_tofs(H, n_paths=5):
    """
    Usa o algoritmo MUSIC (MUltiple SIgnal Classification) para encontrar
    os Tempos de Voo (ToF) com "super-resolução", quebrando o limite do IFFT.
    Isola o sinal direto (Line of Sight) dos ecos nas paredes.
    """
    from scipy.signal import find_peaks

    L = len(H)
    if L < 10:
        return np.array([0.0])

    # 1. Spatial Smoothing (Construção da Matriz de Covariância)
    M = L // 2  # Tamanho da janela de suavização
    X = np.zeros((M, L - M + 1), dtype=complex)
    for i in range(L - M + 1):
        X[:, i] = H[i : i + M]
    R = (X @ X.conj().T) / (L - M + 1)

    # 2. Decomposição em Autovalores e Autovetores
    eigenvalues, eigenvectors = np.linalg.eigh(R)
    
    # Ordenar do maior para o menor (sinal vs ruído)
    idx = np.argsort(np.abs(eigenvalues))[::-1]
    eigenvectors = eigenvectors[:, idx]

    # 3. Separar o subespaço de ruído (descarta os caminhos mais fortes)
    En = eigenvectors[:, n_paths:]

    # 4. Escaneamento do Pseudo-espectro (0 a 300ns = de 0 a 90 metros)
    F_spacing = BANDWIDTH_HZ / 64.0  # Espaçamento típico aprox 312.5 kHz
    taus = np.linspace(0, 300e-9, 300) # 300 pontos de 1ns
    P_music = np.zeros(len(taus))
    
    n_arr = np.arange(M)
    En_EnH = En @ En.conj().T
    
    for i, tau in enumerate(taus):
        # Steering vector (Vetor de fase para essa distância)
        a = np.exp(-1j * 2 * np.pi * F_spacing * n_arr * tau)
        # Denominador do MUSIC (projeção no subespaço de ruído)
        den = np.abs(np.dot(a.conj().T, np.dot(En_EnH, a)))
        P_music[i] = 1.0 / (den + 1e-12)

    # 5. Encontra os picos (tunar proeminência para não perder o sinal de linha de visada mais fraco)
    peaks, _ = find_peaks(P_music, distance=3, prominence=np.max(P_music) * 0.01)
    if len(peaks) == 0:
        return np.array([0.0])
        
    # Pega os maiores picos (refletem os ecos reais do WiFi)
    order = np.argsort(P_music[peaks])[::-1]
    best_taus = taus[peaks[order[:n_paths]]]
    return np.sort(best_taus)


# ═════════════════════════════════════════════
#  3. NEURAL RF SLAM (PyTorch)
# ═════════════════════════════════════════════
if TORCH_OK:
    class NeuralSLAM(nn.Module):
        def __init__(self, n_feat=32, n_va=N_VIRTUAL_ANCHORS):
            super().__init__()
            self.n_va = n_va
            self.encoder = nn.Sequential(
                nn.Linear(n_feat * 2, 128), nn.ReLU(),
                nn.Linear(128, 64),         nn.ReLU(),
                nn.Linear(64, 2)
            )
            self.virtual_anchors = nn.Parameter(torch.randn(n_va, 2) * 2.0)

        def forward(self, feat):
            pos = self.encoder(feat)
            anc = torch.cat([
                torch.zeros(1, 2, device=pos.device),
                self.virtual_anchors
            ], dim=0)
            diff  = pos.unsqueeze(1) - anc.unsqueeze(0)
            dists = torch.norm(diff, dim=2)   # metros — escala real, sem divisão por c
            return pos, dists

    def hungarian_loss(tofs_list, dists_pred):
        """Compara distâncias medidas (ToF × c) com distâncias preditas pela rede."""
        loss = torch.tensor(0.0)
        for i, t_meas in enumerate(tofs_list):
            if len(t_meas) == 0:
                continue
            # Converte ToF (s) → distância (m) para ficar na mesma escala da rede
            dm = torch.tensor(t_meas * SPEED_OF_LIGHT, dtype=torch.float32)
            cost = torch.abs(dm.unsqueeze(1) - dists_pred[i].unsqueeze(0))
            ri, ci = linear_sum_assignment(cost.detach().numpy())
            loss += nn.functional.smooth_l1_loss(
                cost[ri, ci], torch.zeros(len(ri))
            )
        return loss / max(len(tofs_list), 1)


# ═════════════════════════════════════════════
#  4. EKF POR ROBÔ  (estado 3-DOF: x, y, θ)
# ═════════════════════════════════════════════
class RobotEKF:
    """
    Estado: [x, y, θ]  (metros, metros, radianos)
    Predição via modelo cinemático unicycle (vx, vy, omega, dt)
    Atualização de θ  via odometria direta (alta confiança)
    Atualização de xy via medição CSI neural
    """
    def __init__(self):
        self.x   = np.zeros(3)                            # estado
        self.P   = np.diag([1.0, 1.0, np.pi])             # σ_xy≈1m, σ_θ≈√π rad — converge mais rápido
        self.initialized = False
        self.last_t      = None
        self.lock        = threading.Lock()

    # ── Predição ──────────────────────────────
    def predict(self, vx: float, vy: float, omega: float, dt: float):
        """
        Modelo unicycle completo:
          x'  = x  + vx·cos(θ)·dt − vy·sin(θ)·dt
          y'  = y  + vx·sin(θ)·dt + vy·cos(θ)·dt
          θ'  = θ  + ω·dt   ← suavidade temporal; update_theta corrige com medição direta

        Q proporcional a dt.
        """
        if dt <= 0 or dt > 2.0:
            return

        with self.lock:
            th = self.x[2]
            c, s = np.cos(th), np.sin(th)

            F = np.eye(3)
            F[0, 2] = (-vx * s - vy * c) * dt
            F[1, 2] = ( vx * c - vy * s) * dt

            self.x[0] += (vx * c - vy * s) * dt
            self.x[1] += (vx * s + vy * c) * dt
            self.x[2]  = (self.x[2] + omega * dt + np.pi) % (2 * np.pi) - np.pi

            q = dt
            Q = np.diag([
                (EKF_SIGMA_VX    * q) ** 2,
                (EKF_SIGMA_VY    * q) ** 2,
                (EKF_SIGMA_OMEGA * q) ** 2,
            ])
            self.P = F @ self.P @ F.T + Q
            self.initialized = True

    # ── Atualização de θ (odometria direta) ───
    def update_theta(self, th_odom: float, sigma: float = 0.05):
        """
        Corrige θ com a medição direta da odometria do robô.
        H = [0 0 1]  — observamos apenas θ.
        Inovação angular normalizada em [-π, π] para evitar saltos de ±2π.
        """
        with self.lock:
            H = np.array([[0.0, 0.0, 1.0]])
            # Inovação angular normalizada
            inn = (th_odom - self.x[2] + np.pi) % (2 * np.pi) - np.pi
            R = np.array([[sigma ** 2]])
            S = float((H @ self.P @ H.T + R)[0, 0])
            K = (self.P @ H.T / S).flatten()   # shape (3,)

            self.x    = self.x + K * inn
            self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi
            self.P    = (np.eye(3) - np.outer(K, H)) @ self.P
            self.initialized = True

    def update_csi(self, x_meas: float, y_meas: float,
                   sigma: float = EKF_SIGMA_CSI):
        """
        Medição: z = [x_meas, y_meas]
        H = [1 0 0 ; 0 1 0]  (observamos só x e y)
        """
        with self.lock:
            if not self.initialized:
                self.x[0] = x_meas
                self.x[1] = y_meas
                self.initialized = True
                return

            H = np.array([[1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0]])
            z     = np.array([x_meas, y_meas])
            z_hat = H @ self.x
            y_inn = z - z_hat

            R = np.eye(2) * (sigma ** 2)
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)

            # Impede o CSI de alterar o ângulo theta (K para a 3ª linha = 0)
            K[2, :] = 0.0

            self.x = self.x + K @ y_inn
            self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi
            self.P = (np.eye(3) - K @ H) @ self.P

    # ── Pose atual ────────────────────────────
    @property
    def pose(self):
        with self.lock:
            return self.x.copy(), self.P.copy()


# ═════════════════════════════════════════════
#  5. ESTADO POR ROBÔ
# ═════════════════════════════════════════════
class RobotState:
    def __init__(self, robot_id: str):
        self.id              = robot_id
        self.ekf             = RobotEKF()
        self.csi_buffer      = []
        self.sample_count    = 0
        self.last_train      = 0
        self.map_ready       = False
        self.virtual_anchors = np.random.randn(N_VIRTUAL_ANCHORS, 2) * 2.0
        self.model           = None
        self.optimizer       = None
        self.last_publish_ms = 0
        self.last_rssi       = -70
        self.last_mac        = robot_id
        self.map_published   = False      # flag para evitar spam de slam_map

        self.lock            = threading.Lock()

    # ── Inferência (sem treino) ───────────────
    def infer_position(self, feat: np.ndarray):
        """Estima pos via rede ou retorna estado EKF atual"""
        x, _ = self.ekf.pose
        fallback = [float(x[0]), float(x[1])]

        if not TORCH_OK or self.model is None:
            return fallback

        try:
            self.model.eval()
            with torch.no_grad():
                t = torch.tensor(feat, dtype=torch.float32).unsqueeze(0)
                pos, _ = self.model(t)
                return pos[0].numpy().tolist()
        except Exception:
            return fallback

    # ── Treino assíncrono ─────────────────────
    def maybe_train(self):
        with self.lock:
            should = (
                len(self.csi_buffer) >= TRAIN_BUFFER_SIZE and
                self.sample_count - self.last_train >= RETRAIN_EVERY
            )
        if should:
            threading.Thread(
                target=self._train, daemon=True,
                name=f"train-{self.id[:8]}"
            ).start()

    def _train(self):
        if not TORCH_OK:
            return

        with self.lock:
            n   = min(TRAIN_BUFFER_SIZE, len(self.csi_buffer))
            buf = random.sample(self.csi_buffer, n)   # amostras aleatórias → menos overfitting
            self.last_train = self.sample_count

        log.info(f"[{self.id[:8]}] Treinando com {len(buf)} amostras...")
        feats     = np.stack([b[0] for b in buf])
        tofs_list = [b[1] for b in buf]
        pos_ekf   = np.stack([b[2] for b in buf])

        X        = torch.tensor(feats,   dtype=torch.float32)
        P_target = torch.tensor(pos_ekf, dtype=torch.float32)

        if self.model is None:
            self.model     = NeuralSLAM()
            self.optimizer = optim.Adam(self.model.parameters(), lr=1e-3)

        # Alpha adaptativo: quanto mais o robô se moveu, mais peso dá à ancoragem
        pos_var = float(np.var(pos_ekf, axis=0).mean())
        alpha   = float(np.clip(pos_var * 10, 0.01, 0.3))

        self.model.train()
        best = float("inf")
        for _ in range(60):
            self.optimizer.zero_grad()
            pos_pred, dists_pred = self.model(X)

            loss_tof    = hungarian_loss(tofs_list, dists_pred)
            loss_anchor = nn.functional.smooth_l1_loss(pos_pred, P_target)
            loss        = loss_tof + alpha * loss_anchor

            loss.backward()
            self.optimizer.step()
            if loss.item() < best:
                best = loss.item()

        with self.lock:
            va = self.model.virtual_anchors.detach().numpy()
            self.virtual_anchors = va
            self.map_ready       = True
            self.map_published   = False   # mapa mudou → republica uma vez

        log.info(f"[{self.id[:8]}] Treino OK | loss={best:.6f} α={alpha:.3f}")
        log.info(f"[{self.id[:8]}] Âncoras:\n{self.virtual_anchors.round(2)}")


# ═════════════════════════════════════════════
#  6. SERVIDOR MQTT PRINCIPAL
# ═════════════════════════════════════════════
class SLAMServer:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, robot=ROBOT_FILTER):
        self.broker   = broker
        self.port     = port
        self.robot    = robot           # MAC ou "#"
        self.robots   = {}              # mac → RobotState
        self.lock     = threading.Lock()

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect    = self._on_connect
        self.client.on_message    = self._on_message
        self.client.on_disconnect = self._on_disconnect

    # ── MQTT callbacks ────────────────────────
    def _on_connect(self, client, *_):
        prefix = f"/{self.robot}" if self.robot != "#" else ""
        csi_topic  = f"{prefix}/+/CSI"   if self.robot == "#" else f"/{self.robot}/CSI"
        odom_topic = f"{prefix}/+/odom"  if self.robot == "#" else f"/{self.robot}/odom"
        client.subscribe(csi_topic)
        client.subscribe(odom_topic)
        log.info(f"MQTT conectado → broker={self.broker} | ouvindo CSI+odom")

    def _on_disconnect(self, *_):
        log.warning("MQTT desconectado — tentando reconectar...")

    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic          # e.g. /AA:BB:CC/CSI
            parts = topic.strip("/").split("/")
            # /MAC/tipo  → parts = ['MAC', 'tipo']
            if len(parts) < 2:
                return
            mac  = parts[0]
            tipo = parts[-1]           # CSI ou odom
            payload = json.loads(msg.payload.decode())
            robot = self._get_robot(mac)

            if tipo == "CSI":
                self._handle_csi(robot, mac, payload)
            elif tipo == "odom":
                self._handle_odom(robot, payload)
        except Exception as e:
            log.debug(f"Erro mensagem: {e}")

    # ── Gerência de robôs ─────────────────────
    def _get_robot(self, mac: str) -> RobotState:
        with self.lock:
            if mac not in self.robots:
                self.robots[mac] = RobotState(mac)
                log.info(f"Novo robô detectado: {mac}")
            return self.robots[mac]

    # ── Processar CSI ─────────────────────────
    def _handle_csi(self, robot: RobotState, mac: str, payload: dict):
        data = payload.get("data", [])
        rssi = payload.get("rssi", -70)

        if rssi < MIN_RSSI_DBM:
            return   # sinal muito fraco → CSI não confiável

        if len(data) < 4:
            return

        H = parse_csi(data)
        if len(H) < 4:
            return

        tofs = extract_tofs(H, n_paths=N_VIRTUAL_ANCHORS + 1)

        # Features: z-score por canal + normalização L2 global
        H32 = H[:32]
        if len(H32) < 32:
            H32 = np.pad(H32, (0, 32 - len(H32)))
        feat = np.concatenate([
            _zscore(np.real(H32)),
            _zscore(np.imag(H32))
        ]).astype(np.float32)
        feat = feat / (np.linalg.norm(feat) + 1e-6)   # normalização L2

        with robot.lock:
            # Salvar feat, tofs E a posição EKF atual para ancorar a rede neural ao mundo
            current_x, current_y = robot.ekf.pose[0][:2]
            robot.csi_buffer.append((feat, tofs, (current_x, current_y)))
            robot.sample_count += 1
            robot.last_rssi = rssi
            robot.last_mac  = mac

        # 1. Agendar treino se necessário (coleta de buffer inicial)
        robot.maybe_train()

        x, P = robot.ekf.pose
        
        # Só começar a aplicar e publicar o SLAM após a primeira rodada de treino
        if not robot.map_ready:
            if robot.sample_count % 10 == 0:
                log.info(f"[{mac[:11]}] Coletando calibração CSI... ({robot.sample_count}/{TRAIN_BUFFER_SIZE})")
            return

        # 2. Inferência neural → posição CSI
        pos_csi = robot.infer_position(feat)

        # 3. Atualização EKF com medição CSI — sigma cresce com sinal mais fraco
        sigma = EKF_SIGMA_CSI * (1.0 + max(0.0, -rssi - 60) / 20.0)
        
        # Se a odometria informa que o robô está perfeitamente parado, 
        # confiamos nela e ignoramos o ruído do CSI (que fica flutuando 10cm)
        is_stopped = abs(robot.vx_odom) < 0.02 and abs(robot.w_odom) < 0.02
        if is_stopped:
            return  # Não faz update do CSI para não arrastar o robô parado
            
        robot.ekf.update_csi(pos_csi[0], pos_csi[1], sigma=sigma)

        # 4. Publicar resultado EKF
        self._publish(robot, tofs)

    # ── Processar Odometria ───────────────────
    def _handle_odom(self, robot: RobotState, payload: dict):
        """
        O ESP32 publica a odometria no seu próprio frame local, que começa em
        (0,0,0) no boot e pode ser resetado a qualquer momento.
        Extraímos apenas o DELTA incremental (dx, dy, dtheta) entre dois pacotes
        consecutivos e rotacionamos para o frame do robô antes de alimentar o EKF.

        Formato: { "x": float, "y": float, "theta": float, "t": ms }
        """
        x_m  = float(payload.get("x", 0.0))
        y_m  = float(payload.get("y", 0.0))
        th_m = float(payload.get("theta", 0.0))
        t_ms = float(payload.get("t", 0))

        if not hasattr(robot, 'last_odom_state'):
            robot.last_odom_state = (x_m, y_m, th_m, t_ms)
            return

        last_x, last_y, last_th, last_t = robot.last_odom_state

        # Sempre atualiza o estado antes de qualquer verificação,
        # para que o próximo pacote tenha uma base coerente
        robot.last_odom_state = (x_m, y_m, th_m, t_ms)

        raw_dt = (t_ms - last_t) / 1000.0
        if raw_dt <= 0:
            return  # Pacote duplicado ou fora de ordem

        # Limita dt a um intervalo fisicamente razoável
        dt = np.clip(raw_dt, 0.001, 1.0)

        # Deltas no frame local do robô (o (0,0,0) pode flutuar, mas o delta é válido)
        dx  = x_m - last_x
        dy  = y_m - last_y
        dth = (th_m - last_th + np.pi) % (2 * np.pi) - np.pi  # normaliza em [-π, π]

        # Detecção de salto de x/y (reset do robô ou teleporte)
        dist = np.hypot(dx, dy)
        xy_jump    = dist > MAX_ODOM_DIST_M or dist / dt > MAX_ODOM_SPEED_MS
        theta_jump = abs(dth / dt) > MAX_ODOM_OMEGA_RS

        if xy_jump or theta_jump:
            log.warning(
                f"[{robot.id[:11]}] Salto de odometria — "
                f"v={dist/dt:.2f} m/s ω={dth/dt:.2f} rad/s | "
                f"{'xy ' if xy_jump else ''}{'θ ' if theta_jump else ''}ignorado(s)"
            )
            # th_m é medição DIRETA do ângulo do robô (confiável sempre).
            # O jump indica que Δθ/dt foi ruidoso, não que th_m em si é errado.
            # Sem isso, P[2,2] nunca baixa e theta fica preso no valor inicial.
            robot.ekf.update_theta(th_m, sigma=0.05)
            return

        # Rotaciona o delta global para o frame local do robô
        c = np.cos(-last_th)
        s = np.sin(-last_th)
        vx = (dx * c - dy * s) / dt
        vy = (dx * s + dy * c) / dt  # idealmente ~0 para robô diferencial

        omega = dth / dt

        # 1. Prediz posição x,y,θ (omega dá suavidade; update_theta corrige com medição)
        robot.ekf.predict(vx, vy, omega, dt)

        # 2. Corrige θ diretamente com a medição da odometria (alta confiança)
        robot.ekf.update_theta(th_m, sigma=0.05)

        x, P = robot.ekf.pose
        self._publish(robot, tofs=None)

        diff_theta = (x[2] - th_m + np.pi) % (2 * np.pi) - np.pi
        log.info(
            f"[{robot.id[:11]}] Theta Robô: {np.degrees(th_m):.1f}° | "
            f"Theta SLAM (EKF): {np.degrees(x[2]):.1f}° | "
            f"Diferença: {np.degrees(diff_theta):.1f}°"
        )

    # ── Publicar resultado EKF ────────────────
    def _publish(self, robot: RobotState, tofs=None):
        if not robot.map_ready:
            return

        now_ms = int(time.time() * 1000)
        if now_ms - robot.last_publish_ms < EKF_MIN_PUBLISH_MS:
            return  # limita taxa de publicação

        robot.last_publish_ms = now_ms
        x, P = robot.ekf.pose

        # Incerteza posicional (1-sigma em metros)
        sigma_x = float(np.sqrt(max(P[0, 0], 0)))
        sigma_y = float(np.sqrt(max(P[1, 1], 0)))
        sigma_t = float(np.sqrt(max(P[2, 2], 0)))

        msg = {
            "mac":       robot.last_mac,
            "rssi":      robot.last_rssi,
            "pos": {
                "x":     round(float(x[0]), 4),
                "y":     round(float(x[1]), 4),
                "theta": round(float(x[2]), 4),  # radianos
            },
            "cov": {
                "sx":    round(sigma_x, 4),
                "sy":    round(sigma_y, 4),
                "st":    round(sigma_t, 4),
            },
            "tofs_ns":   [round(t * 1e9, 1) for t in tofs] if tofs is not None else [],
            "n_samples": robot.sample_count,
            "map_ready": robot.map_ready,
            "t":         now_ms,
        }
        self.client.publish(f"/{robot.id}/slam", json.dumps(msg), qos=0)

        # Publica mapa somente quando muda (evita spam de 20 Hz)
        if robot.map_ready and not robot.map_published:
            self._publish_map(robot)
            robot.map_published = True

    def _publish_map(self, robot: RobotState):
        with robot.lock:
            anchors = [{"x": float(a[0]), "y": float(a[1])}
                       for a in robot.virtual_anchors]
        msg = {
            "type":    "map",
            "anchors": anchors,
            "t":       int(time.time() * 1000),
        }
        self.client.publish(f"/{robot.id}/slam_map", json.dumps(msg), qos=0)

    # ── Start ─────────────────────────────────
    def run(self):
        log.info("=" * 55)
        log.info("  Neural RF SLAM + EKF Swarm Server")
        log.info(f"  Broker  : {self.broker}:{self.port}")
        log.info(f"  Robôs   : {'TODOS' if self.robot == '#' else self.robot}")
        log.info(f"  PyTorch : {'SIM' if TORCH_OK else 'NÃO (fallback geométrico)'}")
        log.info(f"  EKF σ   : CSI={EKF_SIGMA_CSI}m | vx={EKF_SIGMA_VX}m/s | ω={EKF_SIGMA_OMEGA}rad/s")
        log.info("=" * 55)

        self.client.connect(self.broker, self.port, keepalive=60)
        # Reconexão automática
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.client.loop_forever()


# ═════════════════════════════════════════════
#  UTILS
# ═════════════════════════════════════════════
def _zscore(v: np.ndarray) -> np.ndarray:
    s = v.std()
    return (v - v.mean()) / (s + 1e-8) if s > 0 else v - v.mean()


# ═════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════
if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Neural RF SLAM + EKF Server")
    p.add_argument("--broker", default=MQTT_BROKER, help="IP do broker MQTT")
    p.add_argument("--port",   default=MQTT_PORT,   type=int)
    p.add_argument("--robot",  default=ROBOT_FILTER, help="MAC do robô ou '#' para todos")
    p.add_argument("--sigma-csi",   default=EKF_SIGMA_CSI,   type=float, help="Ruído CSI (m)")
    p.add_argument("--sigma-vx",    default=EKF_SIGMA_VX,    type=float, help="Ruído vel X (m/s)")
    p.add_argument("--sigma-omega", default=EKF_SIGMA_OMEGA, type=float, help="Ruído angular (rad/s)")
    p.add_argument("--debug",  action="store_true")
    args = p.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    EKF_SIGMA_CSI   = args.sigma_csi
    EKF_SIGMA_VX    = args.sigma_vx
    EKF_SIGMA_OMEGA = args.sigma_omega

    SLAMServer(broker=args.broker, port=args.port, robot=args.robot).run()
