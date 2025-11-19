#include "diff_car.h"
#include <algorithm>

#ifdef ENCODER_SIMPLE

void IRAM_ATTR left_encoder_isr() {
  unsigned long now = micros();
  unsigned long prev = diffCar.left_last_pulse_us;
  // debounce por tempo mínimo entre pulsos
  if (prev == 0 || (now - prev) > DEBOUNCE_US) {
    diffCar.left_last_pulse_us = now;  
    diffCar.left_last_pulse_ms = millis();
    diffCar.encoder_left_count++;
  }
}

void IRAM_ATTR right_encoder_isr() {
  unsigned long now = micros();
  unsigned long prev = diffCar.right_last_pulse_us;
  if (prev == 0 || (now - prev) > DEBOUNCE_US) {
    diffCar.right_last_pulse_us = now;
    diffCar.right_last_pulse_ms = millis();
    diffCar.encoder_right_count++;
  }
}

void DiffCar::setup_encoder() {
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_1), right_encoder_isr, RISING);
  last_sample_time_ms = millis();
}

// função simples para mediana de 3 (ajusta se MEDIAN_WINDOW mudar)
unsigned long median3(unsigned long a, unsigned long b, unsigned long c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void DiffCar::velocity_update() {
    unsigned long now_ms = millis();

    // Copia segura das variáveis atualizadas nos ISRs
    noInterrupts();
    unsigned long left_last_interval_us  = this->left_last_interval_us;
    unsigned long right_last_interval_us = this->right_last_interval_us;
    unsigned long left_last_ms  = this->left_last_pulse_ms;
    unsigned long right_last_ms = this->right_last_pulse_ms;
    long left_count_now  = this->encoder_left_count;
    long right_count_now = this->encoder_right_count;
    interrupts();

    // ---- MÉTODO DE INTERVALO DIRETO
    float left_freq_interval  = (left_last_interval_us  > 0) ? 1e6f / left_last_interval_us  : 0;
    float right_freq_interval = (right_last_interval_us > 0) ? 1e6f / right_last_interval_us : 0;

    // ---- MÉTODO DE JANELA
    float left_freq_window = 0, right_freq_window = 0;

    if (now_ms - last_sample_time_ms >= SAMPLE_MS) {

        unsigned long dt_ms = now_ms - last_sample_time_ms;
        long delta_left  = left_count_now  - last_count_left_snapshot;
        long delta_right = right_count_now - last_count_right_snapshot;

        if (dt_ms > 0) {
            float dt_s = dt_ms / 1000.0f;
            left_freq_window  = delta_left  / dt_s;
            right_freq_window = delta_right / dt_s;
        }

        // ---- SELEÇÃO ENTRE JANELA E INTERVALO
        const int threshold = 1;  // recomendo este valor para não perder baixa velocidade

        float left_freq_combined  = (delta_left  >= threshold) ? left_freq_window  : left_freq_interval;
        float right_freq_combined = (delta_right >= threshold) ? right_freq_window : right_freq_interval;

        // ---- EMA
        left_freq_filtered  = EMA_ALPHA * left_freq_combined  + (1 - EMA_ALPHA) * left_freq_filtered;
        right_freq_filtered = EMA_ALPHA * right_freq_combined + (1 - EMA_ALPHA) * right_freq_filtered;

        // Atualiza snapshots
        last_count_left_snapshot  = left_count_now;
        last_count_right_snapshot = right_count_now;
        last_sample_time_ms = now_ms;
    }

    // ---- TIMEOUT DE PARADA
    if (now_ms - left_last_ms > NO_PULSE_TIMEOUT_MS) {
        left_freq_filtered = 0;
        left_stopped = true;
    } else left_stopped = false;

    if (now_ms - right_last_ms > NO_PULSE_TIMEOUT_MS) {
        right_freq_filtered = 0;
        right_stopped = true;
    } else right_stopped = false;

    // ---- DEADZONE
    if (left_freq_filtered < MIN_PULSES_PER_S)  left_freq_filtered = 0;
    if (right_freq_filtered < MIN_PULSES_PER_S) right_freq_filtered = 0;

    float left_pulses_s = left_freq_filtered;
    float right_pulses_s = right_freq_filtered;

    // ---- Cálculo correto de m/s
    float meters_per_pulse = WHEEL_CIRCUMFERENCE_M / PULSES_PER_REV;

    left_velocity_ms  = left_pulses_s  * meters_per_pulse;
    right_velocity_ms = right_pulses_s * meters_per_pulse;
}


void DiffCar::debug_encoder() {
    //Serial.print("encoder Left: "); Serial.print(diffCar.encoder_left_count);
      Serial.print("Left Velocity (m/s): ");
      Serial.print(left_velocity_ms, 2);
      Serial.print("| Left PWM: ");
      Serial.print(left_motor_pwm, 2);
      Serial.print("|| Right Velocity (m/s): ");
      Serial.print(right_velocity_ms, 2);
      Serial.print("| Right PWM: ");
      Serial.println(right_motor_pwm, 2);


}




#endif