
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

    if (dt <= 0.0f || dt > 0.5f) return;

    static uint32_t last_left_pulses = 0;
    static uint32_t last_right_pulses = 0;

    noInterrupts();
    uint32_t current_left_pulses = left_total_pulses;
    uint32_t current_right_pulses = right_total_pulses;
    interrupts();

    int32_t dlp = current_left_pulses  - last_left_pulses;
    int32_t drp = current_right_pulses - last_right_pulses;

    last_left_pulses  = current_left_pulses;
    last_right_pulses = current_right_pulses;

    float dl = dlp * CENTIMETER_PER_PULSE * (left_motor_dir  ? 1.0f : -1.0f);
    float dr = drp * CENTIMETER_PER_PULSE * (right_motor_dir ? 1.0f : -1.0f);

    float dc = (dl + dr) * 0.5f;
    float dtheta = (dr - dl) / WHEEL_BASE_CM;

    float v = dc / dt;

    // ===== ODOMETRIA PURA =====
    float theta_mid = odom_enc.theta + 0.5f * dtheta;

    odom_enc.x += dc * cosf(theta_mid);
    odom_enc.y += dc * sinf(theta_mid);
    odom_enc.theta = wrapAngleD(odom_enc.theta + dtheta);

    odom_enc.linear_velocity = v;
    odom_enc.angular_velocity = dtheta / dt;

    // ===== EKF =====
    ekf_update(dc, dtheta, v, dt);
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
