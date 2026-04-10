#include "diff_car.h"
#include <algorithm>
#include "driver/pcnt.h"

#define PCNT_LEFT_UNIT   PCNT_UNIT_0
#define PCNT_RIGHT_UNIT  PCNT_UNIT_1

#define PCNT_H_LIM  32767
#define PCNT_L_LIM -32768

#ifdef ENCODER_SIMPLE


void DiffCar::setup_encoder() {
    pinMode(ENCODER_A_1, INPUT_PULLUP);
    pinMode(ENCODER_B_1, INPUT_PULLUP);

    // ── LEFT ─────────────────────────────────────────────────────
    pcnt_config_t left = {};
    left.unit           = PCNT_LEFT_UNIT;
    left.channel        = PCNT_CHANNEL_0;
    left.pulse_gpio_num = ENCODER_A_1;
    left.ctrl_gpio_num  = PCNT_PIN_NOT_USED;

    // Chave óptica 1 canal: conta só borda de SUBIDA
    // (cada furo/dente passa = 1 pulso, sem dobrar a contagem)
    left.pos_mode  = PCNT_COUNT_INC;  // sobe na rising edge
    left.neg_mode  = PCNT_COUNT_DIS;  // ignora falling edge

    left.lctrl_mode = PCNT_MODE_KEEP;
    left.hctrl_mode = PCNT_MODE_KEEP;
    left.counter_h_lim = PCNT_H_LIM;
    left.counter_l_lim = PCNT_L_LIM;
    pcnt_unit_config(&left);

    // Chave óptica: 100–300 ciclos (1.25–3.75μs) é suficiente
    // 1000 ciclos (12.5μs) pode perder pulsos acima de ~40kHz
    pcnt_set_filter_value(PCNT_LEFT_UNIT, 500);
    pcnt_filter_enable(PCNT_LEFT_UNIT);

    pcnt_counter_pause(PCNT_LEFT_UNIT);
    pcnt_counter_clear(PCNT_LEFT_UNIT);
    pcnt_counter_resume(PCNT_LEFT_UNIT);

    // ── RIGHT ────────────────────────────────────────────────────
    pcnt_config_t right = left;
    right.unit           = PCNT_RIGHT_UNIT;
    right.pulse_gpio_num = ENCODER_B_1;
    pcnt_unit_config(&right);

    pcnt_set_filter_value(PCNT_RIGHT_UNIT, 500);
    pcnt_filter_enable(PCNT_RIGHT_UNIT);

    pcnt_counter_pause(PCNT_RIGHT_UNIT);
    pcnt_counter_clear(PCNT_RIGHT_UNIT);
    pcnt_counter_resume(PCNT_RIGHT_UNIT);

    pcnt_get_counter_value(PCNT_LEFT_UNIT,  &left_count_snapshot);
    pcnt_get_counter_value(PCNT_RIGHT_UNIT, &right_count_snapshot);

    Serial.println("PCNT encoders initialized (Single Edge, optical sensor).");
}

// ── Kalman 1D com ganho adequado para velocidade ─────────────────────────
// Q: quanto a velocidade pode mudar entre amostras (dinâmica do robô)
// R: ruído do sensor (chave óptica é limpa, mas tem quantização)
float DiffCar::kalman_update(float measurement, float &x, float &P) {
    const float Q = 50.0f;   // aceita mudanças rápidas de velocidade
    const float R = 2000.0f;   // chave óptica tem pouco ruído (R/Q = 3x)

    float P_pred = P + Q;
    float K      = P_pred / (P_pred + R);

    x = x + K * (measurement - x);
    P = (1.0f - K) * P_pred;

    return x;
}

void DiffCar::velocity_update() {
    unsigned long now_us = micros();
    if (now_us - last_vel_update_us < LOOP_FAST_US) return;

    unsigned long dt_us = now_us - last_vel_update_us;
    last_vel_update_us  = now_us;
    float dt = dt_us * 1e-6f;

    int16_t left_now  = 0;
    int16_t right_now = 0;
    pcnt_get_counter_value(PCNT_LEFT_UNIT,  &left_now);
    pcnt_get_counter_value(PCNT_RIGHT_UNIT, &right_now);

    int16_t dleft  = left_now  - left_count_snapshot;
    int16_t dright = right_now - right_count_snapshot;
    left_count_snapshot  = left_now;
    right_count_snapshot = right_now;

    // Pulsos/s (sempre positivo — direção vem do motor_dir)
    float left_freq  = fabsf((float)dleft  / dt);
    float right_freq = fabsf((float)dright / dt);

    // ── Anti-spike ───────────────────────────────────────────────
    // Limita variação máxima entre amostras
    const float max_delta = 1500.0f; // pulsos/s — ajuste se necessário
    left_freq  = constrain(left_freq,
                           left_freq_filtered  - max_delta,
                           left_freq_filtered  + max_delta);
    right_freq = constrain(right_freq,
                           right_freq_filtered - max_delta,
                           right_freq_filtered + max_delta);

    // ── Kalman ───────────────────────────────────────────────────
    left_freq_filtered  = kalman_update(left_freq,  left_kalman_x,  left_kalman_P);
    right_freq_filtered = kalman_update(right_freq, right_kalman_x, right_kalman_P);

    // ── Deadzone (parado real) ───────────────────────────────────
    if (left_freq_filtered  < MIN_PULSES) left_freq_filtered  = 0.0f;
    if (right_freq_filtered < MIN_PULSES) right_freq_filtered = 0.0f;

    // ── Conversão para m/s com sinal de direção ──────────────────
    // A chave óptica não detecta direção — usamos o comando do motor
    left_velocity_cms  = left_freq_filtered  * CENTIMETER_PER_PULSE
                        * (left_motor_dir  ? 1.0f : -1.0f);
    right_velocity_cms = right_freq_filtered * CENTIMETER_PER_PULSE
                        * (right_motor_dir ? 1.0f : -1.0f);
}

void DiffCar::debug_encoder() {
      Serial.print("encoder Left: "); Serial.println(diffCar.left_count_snapshot);
      Serial.print("LV (cm/s): ");
      Serial.print(left_velocity_cms, 2);
      Serial.print("Target LV: ");
      Serial.print(left_velocity_target, 2);
      Serial.print("| Lp: ");
      Serial.print(left_motor_pwm, 2);
}


void DiffCar::debug_raw_pins() {
    Serial.print("Raw A: ");
    Serial.print(digitalRead(ENCODER_A_1));
    Serial.print(" | Raw B: ");
    Serial.println(digitalRead(ENCODER_B_1));
}

#endif
