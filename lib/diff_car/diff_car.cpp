
#include "diff_car.h"

#include <stdio.h>
#include <math.h>

DiffCar::DiffCar() {}

static double wrapAngleD(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

void DiffCar::setup() {
  setup_encoder();
  setup_h_bridge();
  setup_mpu();
  setup_leds();
  update_leds(0,1,0,1,0,1,0,1);
}




void DiffCar::encoder_odometry_update() {
    static unsigned long last_odom_time = micros();
    unsigned long current_time = micros();
    float dt = (current_time - last_odom_time) * 1e-6f;
    last_odom_time = current_time;

    float vl = left_motor_dir ? left_velocity_cms : -left_velocity_cms;
    float vr = right_motor_dir ? right_velocity_cms : -right_velocity_cms; 

    float v = (vr + vl) * 0.5f;             
    float omega = (vr - vl) * (1.0f / WHEEL_BASE_CM);              
    
    odom_enc.linear_velocity = v;
    odom_enc.angular_velocity = omega;

    // Failsafe para pauses da Task ou Boot inicial (evita pulos abruptos)
    if (dt > 0.5f) dt = 0.0f;

    // ==============================================================
    // EKF - Filtro de Kalman Estendido 4D
    // Estados: X = [ x, y, theta, velocidade_linear ]
    // Controles: u = [ omega_encoder, aceleração_IMU ]
    // Sensores (Medição): Z = [ theta_IMU, velocidade_Encoder ]
    // ==============================================================
    static float X_k[4] = {0.0f, 0.0f, 0.0f, 0.0f}; 
    static float P_k[4][4] = {
        {1.0f, 0, 0, 0},
        {0, 1.0f, 0, 0},
        {0, 0, 1.0f, 0},
        {0, 0, 0, 1.0f}
    };

    // Ajuste aqui os Ruídos (Q = Confiança no Movimento, R = Confiança no Sensor)
    constexpr float Q[4] = {
    50.0f,    // x       [cm²]      — incerteza do modelo cinemático
    50.0f,    // y       [cm²]      — idem
    1e-4f,    // theta   [rad²]     — drift real do giroscópio MPU6050
    100.0f    // v       [cm²/s²]   — acelerômetro + escorregamento de roda
    };
    constexpr float R[2] = {
    0.02f,    // theta_IMU  [rad²]  — drift do yaw MPU6050 (sem mag = ruim)
    50.0f     // v_encoder  [cm²/s²]— encoder é confiável, use valor baixo
    };     

    // 1. PREDIÇÃO (Prediction Step)
    float x_pred     = X_k[0] + X_k[3] * cosf(X_k[2]) * dt;
    float y_pred     = X_k[1] + X_k[3] * sinf(X_k[2]) * dt;
    float theta_pred = wrapAngleD(X_k[2] + omega * dt); 
    // aceleração convertida de m/s² para cm/s² (100x)
    float v_pred     = X_k[3] + (accel_d[1] * 100.0f) * dt;       

    float X_pred[4] = {x_pred, y_pred, theta_pred, v_pred};

    // Matriz Jacobiana F (Derivadas parciais do modelo para atualização da incerteza)
    float F[4][4] = {
        {1.0f, 0.0f, -X_k[3] * sinf(X_k[2]) * dt,  cosf(X_k[2]) * dt},
        {0.0f, 1.0f,  X_k[3] * cosf(X_k[2]) * dt,  sinf(X_k[2]) * dt},
        {0.0f, 0.0f,  1.0f,                        0.0f},
        {0.0f, 0.0f,  0.0f,                        1.0f}
    };

    // Covariância predita: P_pred = F * P_k * F^T + Q
    float FP[4][4] = {0};
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            for(int k=0; k<4; k++)
                FP[i][j] += F[i][k] * P_k[k][j];

    float P_pred[4][4] = {0};
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++) {
            for(int k=0; k<4; k++)
                P_pred[i][j] += FP[i][k] * F[j][k]; // Multiplica por F transposta
            if(i == j) P_pred[i][j] += Q[i];        // Soma Ruído de Processo
        }

    
    // Inovação y = Z - H*X_pred
    float y_inov[2];
    y_inov[0] = wrapAngleD(ypr_d[0] - X_pred[2]); 
    y_inov[1] = v - X_pred[3];                    


    // Covariância da Inovação: S = H * P_pred * H^T + R
    float S[2][2];
    S[0][0] = P_pred[2][2] + R[0];
    S[0][1] = P_pred[2][3];
    S[1][0] = P_pred[3][2];
    S[1][1] = P_pred[3][3];

    // Inversa de S (S_inv)
    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    float S_inv[2][2] = {0};
    if (fabs(det) > 1e-6f) { 
        S_inv[0][0] =  S[1][1] / det;
        S_inv[0][1] = -S[0][1] / det;
        S_inv[1][0] = -S[1][0] / det;
        S_inv[1][1] =  S[0][0] / det;
    }

    // Ganho de Kalman: K = P_pred * H^T * S_inv
    float PHT[4][2];
    for(int i=0; i<4; i++) {
        PHT[i][0] = P_pred[i][2]; // Extrai coluna Theta (2)
        PHT[i][1] = P_pred[i][3]; // Extrai coluna Vel. Linear (3)
    }
    
    float K_gain[4][2] = {0};
    for(int i=0; i<4; i++)
        for(int j=0; j<2; j++)
            for(int k=0; k<2; k++)
                K_gain[i][j] += PHT[i][k] * S_inv[k][j];

    // Corrige os Estados: X_k = X_pred + K * y_inov
    for(int i=0; i<4; i++) {
        X_k[i] = X_pred[i] + K_gain[i][0]*y_inov[0] + K_gain[i][1]*y_inov[1];
    }
    X_k[2] = wrapAngleD(X_k[2]); 

    // Atualiza a Covariância do Modelo: P_k = (I - K*H) * P_pred
    float I_KH[4][4] = {0};
    for(int i=0; i<4; i++) {
        I_KH[i][i] = 1.0f;
        I_KH[i][2] -= K_gain[i][0];
        I_KH[i][3] -= K_gain[i][1];
    }
    
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++) {
            P_k[i][j] = 0;
            for(int k=0; k<4; k++)
                P_k[i][j] += I_KH[i][k] * P_pred[k][j];
        }

    // Exportação dos Estados Corrigidos pelo Filtro
    odom_real.x = X_k[0];
    odom_real.y = X_k[1];
    odom_real.theta = X_k[2];
    odom_real.linear_velocity = X_k[3];
}

void DiffCar::navigate_to_target_pure_pursuit(
    float target_x,
    float target_y,
    float target_theta,
    float precision,
    float target_velocity)
{
    float dx = target_x - odom_real.x;
    float dy = target_y - odom_real.y;
    float distance = sqrt(dx * dx + dy * dy);

    // =========================
    // FILTRO EMA NO ANGULAR VELOCITY (encoder com ruído)
    // =========================
    static float w_filtered = 0.0f;
    const float ALPHA_W = 0.15f;  // mais lento = mais suave; ajuste entre 0.1..0.25
    w_filtered = ALPHA_W * odom_enc.angular_velocity + (1.0f - ALPHA_W) * w_filtered;

    // =========================
    // ORIENTAÇÃO FINAL (OPCIONAL)
    // =========================
    bool use_heading = !isnan(target_theta);
    float final_error = use_heading ? wrapAngleD(target_theta - odom_real.theta) : 0.0f;

    // =========================
    // LOOKAHEAD DINÂMICO
    // =========================
    float base_Ld = distance * 0.5f;
    float Ld = base_Ld + fabs(target_velocity) * 0.3f;
    Ld = constrain(Ld, 20.0f, 120.0f);

    if (distance < 1e-5f) distance = 1e-5f;

    float look_x = odom_real.x + (dx / distance) * Ld;
    float look_y = odom_real.y + (dy / distance) * Ld;

    float dx_r = look_x - odom_real.x;
    float dy_r = look_y - odom_real.y;

    float x_r =  cos(odom_real.theta) * dx_r + sin(odom_real.theta) * dy_r;
    float y_r = -sin(odom_real.theta) * dx_r + cos(odom_real.theta) * dy_r;

    // =========================
    // CURVATURA
    // =========================
    float curvature = (2.0f * y_r) / (Ld * Ld);
    curvature = constrain(curvature, -4.0f, 4.0f);

    if (use_heading) {
        float heading_weight = exp(-distance * 1.5f);
        curvature += final_error * heading_weight;
    }

    // =========================
    // ÂNGULO DE CAMINHO E HISTERESE
    // =========================
    float path_angle       = atan2(dy, dx);
    float angle_error_path = wrapAngleD(path_angle - odom_real.theta);

    // Histerese estreitada: entra em 40°, SAI em 15° (evita flap entre modos)
    static bool aligning_in_place = false;
    if (distance > precision && fabs(angle_error_path) > 0.70f) {  // ~40°
        aligning_in_place = true;
    } else if (fabs(angle_error_path) < 0.26f) {                   // ~15°
        aligning_in_place = false;
    }

    // =========================
    // VELOCIDADE LINEAR
    // =========================
    float v_cmd = 0.0f;

    if (aligning_in_place) {
        v_cmd = 0.0f;
    } else {
        v_cmd = target_velocity * fmin(1.0f, distance / 50.0f);
        v_cmd *= 1.0f / (1.0f + fabs(angle_error_path) * 2.0f);

        float slow_factor = constrain(distance / (precision + 0.01f), 0.0f, 1.0f);
        v_cmd *= slow_factor;

        if (fabs(v_cmd) < VEL_MIN_LIN)
            v_cmd = copysign(VEL_MIN_LIN * slow_factor, v_cmd);

        v_cmd = constrain(v_cmd, -VEL_MAX_LIN, VEL_MAX_LIN);
    }

    // ============================================
    // LIMITADOR DE ACELERAÇÃO LINEAR
    // ============================================
    static float last_v_cmd = 0.0f;
    static unsigned long last_accel_time = micros();
    unsigned long now_accel = micros();
    float dt_accel = (now_accel - last_accel_time) * 1e-6f;
    last_accel_time = now_accel;

    if (dt_accel > 0.0f && dt_accel < 0.5f) {
        float max_dv = 50.0f * dt_accel;
        v_cmd = constrain(v_cmd, last_v_cmd - max_dv, last_v_cmd + max_dv);
    }
    last_v_cmd = v_cmd;

    // =========================
    // VELOCIDADE ANGULAR (PD com w filtrado)
    // =========================
    float w_cmd = 0.0f;

    if (aligning_in_place) {
        // PD puro: proporcional ao erro, derivativo no w real (filtrado)
        w_cmd = 1.8f * angle_error_path - 0.8f * w_filtered;

        // Banda morta absoluta: abaixo de 3° não aplica torque (corta tremida residual)
        if (fabs(angle_error_path) < 0.05f) {
            w_cmd = 0.0f;
        }
    } else {
        // Controle polar suave durante navegação
        w_cmd = 0.4f * angle_error_path - 0.3f * w_filtered;
    }

    // VEL_MIN_ANG só força se o erro ainda é relevante; caso contrário zera
    // (ausência desse 'else w=0' era a causa principal da tremida residual)
    if (fabs(w_cmd) < VEL_MIN_ANG) {
        if (fabs(angle_error_path) > 0.08f)
            w_cmd = copysign(VEL_MIN_ANG, w_cmd);
        else
            w_cmd = 0.0f;
    }

    w_cmd = constrain(w_cmd, -VEL_MAX_ANG, VEL_MAX_ANG);

    // ============================================
    // LIMITADOR DE ACELERAÇÃO ANGULAR
    // ============================================
    static float last_w_cmd = 0.0f;
    if (dt_accel > 0.0f && dt_accel < 0.5f) {
        float max_dw = 10.0f * dt_accel;
        w_cmd = constrain(w_cmd, last_w_cmd - max_dw, last_w_cmd + max_dw);
    }
    last_w_cmd = w_cmd;

    // =========================
    // CINEMÁTICA DIFERENCIAL
    // =========================
    float L  = WHEEL_BASE_CM;
    float vl = v_cmd - (w_cmd * L * 0.5f);
    float vr = v_cmd + (w_cmd * L * 0.5f);

    float max_vel = fmax(fabs(vl), fabs(vr));
    if (max_vel > VEL_MAX_LIN) {
        float scale = VEL_MAX_LIN / max_vel;
        vl *= scale;
        vr *= scale;
    }

    left_velocity_target  = vl;
    right_velocity_target = vr;

    // =========================
    // FINALIZAÇÃO
    // =========================
    bool reached_position = (distance < precision);
    bool reached_heading  = (!use_heading || fabs(final_error) < 0.05f);

    if (reached_position && reached_heading) {
        left_velocity_target  = 0.0f;
        right_velocity_target = 0.0f;
        mode = 0;
    }
}


void DiffCar::mover_distancia(float centimetros, float velocidade_linear) {
    float x_inicial = odom_real.x;
    float y_inicial = odom_real.y;
    float ang_inicial = odom_real.theta; // Usa a IMU como referência de rumo (Yaw)
    
    // Converte a intenção linear baseada no parâmetro positivo/negativo "centimetros"
    float dist_alvo = fabs(centimetros);
    float vel = (centimetros < 0.0f) ? -fabs(velocidade_linear) : fabs(velocidade_linear);
    
    float dist_percorrida = 0.0f;
    
    // Control loop bloqueante assíncrono à thread principal da aplicação
    while (dist_percorrida < dist_alvo) {
        
        // Mantém a sensoriamento girando em caso do "delay" impedir a main() global
        velocity_update();
        mpu_update();
        encoder_odometry_update();

        // Pitágoras para calcular o quanto se moveu no mundo real (em metros)
        float dx = odom_real.x - x_inicial;
        float dy = odom_real.y - y_inicial;
        dist_percorrida = sqrt(dx*dx + dy*dy);
        
        // Rampa de Desaceleração suave nos últimos 15 cm 
        // para dar espaço à frenagem e evitar "skipping" sobre o ponto alvo
        float vel_calc = vel;
        float erro_dist = dist_alvo - dist_percorrida;
        if (erro_dist < 15.0f) {
            vel_calc = vel * (erro_dist / 15.0f);
            
            // Garante que não zera de vez e não tenha força para rolar a bucha do pneu, travando no caminho
            if (fabs(vel_calc) < 15.0f) {
                vel_calc = copysign(15.0f, vel); 
            }
        }
        
        manter_rumo(vel_calc, ang_inicial);
        
        // Comando do driver para acionar PWMs novos localmente na Task
        handler_motor();
        delay(10); // Pausa de 10ms (100Hz) igual aos ciclos de telemetria base
    }
    
    // Chegou no alvo: Corta os motores imediatamente para estancar atrito
    left_velocity_target = 0.0f;
    right_velocity_target = 0.0f;
    handler_motor(); // Força update da ponte H uma última vez
}

void DiffCar::manter_rumo(float velocidade_linear, float angulo_desejado) {
    // 1. Cálculo do Erro
    float erro_theta = wrapAngleD(angulo_desejado - ypr_d[0]);

    // 2. Parâmetros do PID (Ajuste esses valores!)
    static float erro_acumulado = 0;
    float Kp = 5.0f;   // Resposta imediata
    float Ki = 0.05f;  // Corrige desvios persistentes
    
    // 3. Cálculo do Termo Integral com Anti-Windup
    // Só acumulamos o erro se o robô não estiver saturado (PWM no máximo)
    erro_acumulado += erro_theta;
    
    // Limita o erro acumulado (Clamping) para evitar o "Windup"
    erro_acumulado = constrain(erro_acumulado, -0.5f, 0.5f); 

    // 4. Cálculo da Velocidade Angular (w)
    float w_correcao = (erro_theta * Kp) + (erro_acumulado * Ki);
    
    // Limita a correção para não girar rápido demais
    w_correcao = constrain(w_correcao, -VEL_MAX_ANG, VEL_MAX_ANG);

    // 5. Mixagem nos motores (Cinemática Diferencial)
    float L = WHEEL_BASE_CM;
    left_velocity_target  = velocidade_linear - (w_correcao * L * 0.5f);
    right_velocity_target = velocidade_linear + (w_correcao * L * 0.5f);

    // Se o alvo for parar, zeramos o acumulador para a próxima manobra
    if (velocidade_linear == 0 && fabs(erro_theta) < 0.05f) {
        erro_acumulado = 0;
    }
}

void DiffCar::debug_nav() {
    Serial.print("Odom x: ");
    Serial.print(odom_real.x, 4);
    Serial.print(" | Odom y: ");
    Serial.print(odom_real.y, 4);
    Serial.print(" | Odom theta: ");
    Serial.print(odom_real.theta, 4);
    Serial.print(" | Error x: ");
    Serial.print(odom_real.x - target_x, 4);
    Serial.print(" | Error y: ");
    Serial.println(odom_real.y - target_y, 4);
    Serial.print(" | Error theta: ");
    Serial.println(odom_real.theta - target_theta, 4);
}
