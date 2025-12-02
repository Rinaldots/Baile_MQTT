#include "diff_car.h"
#include <algorithm>

#ifdef ENCODER_SIMPLE

void IRAM_ATTR left_encoder_isr() {
    unsigned long t = micros();
    // Se já houve pulso anterior, calcule intervalo e aplique debounce no intervalo
    if (diffCar.left_prev_pulse_us > 0) {
        unsigned long interval = t - diffCar.left_prev_pulse_us;
        if (interval > DEBOUNCE_US) {
            diffCar.left_last_interval_us = interval;
        }
        // se interval <= DEBOUNCE_US, ignoramos (possível bounce)
    }
    diffCar.left_prev_pulse_us = t;
    diffCar.left_last_pulse_time_ms = millis();
    diffCar.encoder_left_count++;
}

void IRAM_ATTR right_encoder_isr() {
    unsigned long t = micros();
    if (diffCar.right_prev_pulse_us > 0) {
        unsigned long interval = t - diffCar.right_prev_pulse_us;
        if (interval > DEBOUNCE_US) {
            diffCar.right_last_interval_us = interval;
        }
    }
    diffCar.right_prev_pulse_us = t;
    diffCar.right_last_pulse_time_ms = millis();
    diffCar.encoder_right_count++;
}

void DiffCar::setup_encoder() {
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_1), right_encoder_isr, RISING);
}



void DiffCar::velocity_update() {
    unsigned long now_ms = millis();

    noInterrupts();
    unsigned long left_interval_us  = left_last_interval_us;
    unsigned long right_interval_us = right_last_interval_us;
    unsigned long left_last_ms  = left_last_pulse_time_ms;
    unsigned long right_last_ms = right_last_pulse_time_ms;
    interrupts();

    float left_freq = (left_interval_us  > 0) ? (1e6f / left_interval_us)  : 0.0f;
    float right_freq = (right_interval_us > 0) ? (1e6f / right_interval_us) : 0.0f;

    if (now_ms - left_last_ms > TIMEOUT_MS)  left_freq = 0;
    if (now_ms - right_last_ms > TIMEOUT_MS) right_freq = 0;

    left_freq_raw  = left_freq;
    right_freq_raw = right_freq;

    // Histerese + EMA (como já tinha)
    if (left_freq_filtered == 0) {
        if (left_freq > HYST_UP) left_freq_filtered = left_freq;
    } else {
        if (left_freq < HYST_DOWN) left_freq_filtered = 0;
        else left_freq_filtered = EMA_ALPHA * left_freq + (1 - EMA_ALPHA) * left_freq_filtered;
    }

    if (right_freq_filtered == 0) {
        if (right_freq > HYST_UP) right_freq_filtered = right_freq;
    } else {
        if (right_freq < HYST_DOWN) right_freq_filtered = 0;
        else right_freq_filtered = EMA_ALPHA * right_freq + (1 - EMA_ALPHA) * right_freq_filtered;
    }

    if (left_freq_filtered < MIN_PULSES)  left_freq_filtered = 0;
    if (right_freq_filtered < MIN_PULSES) right_freq_filtered = 0;

    left_velocity_ms  = left_freq_filtered  * METER_PER_PULSE;
    right_velocity_ms = right_freq_filtered * METER_PER_PULSE;
    
}

void DiffCar::debug_encoder() {
      Serial.print("encoder Left: "); Serial.println(diffCar.encoder_left_count);
      Serial.print("LV (m/s): ");
      Serial.print(left_velocity_ms, 2);
      Serial.print("Target LV: ");
      Serial.print(left_velocity_target, 2);
      Serial.print("| Lp: ");
      Serial.print(left_motor_pwm, 2);
      Serial.print("|| RV (m/s): ");
      Serial.print(right_velocity_ms, 2);
      Serial.print("Target RV: ");
      Serial.print(right_velocity_target, 2);
      Serial.print("| Rp: ");
      Serial.print(right_motor_pwm, 2);
      Serial.print("| PID left: ");
      Serial.print(left_pid_output, 2);
      Serial.print("| PID right: ");
      Serial.println(right_pid_output, 2);

}




#endif