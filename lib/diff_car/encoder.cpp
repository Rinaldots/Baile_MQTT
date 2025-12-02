#include "diff_car.h"
#include <algorithm>
#include "driver/pcnt.h"

#define PCNT_LEFT_UNIT   PCNT_UNIT_0
#define PCNT_RIGHT_UNIT  PCNT_UNIT_1

#define PCNT_H_LIM  32767
#define PCNT_L_LIM -32768

#ifdef ENCODER_SIMPLE


void DiffCar::setup_encoder() {
    // 1. Configure GPIOs (Explicitly)
    // Ensure you are NOT using GPIO 34-39 unless you have external pull-ups!
    pinMode(ENCODER_A_1, INPUT_PULLUP);
    pinMode(ENCODER_B_1, INPUT_PULLUP);

    // --- LEFT UNIT SETUP ---
    pcnt_config_t left = {};
    left.unit = PCNT_LEFT_UNIT;
    left.channel = PCNT_CHANNEL_0;
    left.pulse_gpio_num = ENCODER_A_1;
    left.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    
    // CHANGE: Count on both edges for higher resolution and better detection
    left.pos_mode = PCNT_COUNT_INC; // Count up on rising
    left.neg_mode = PCNT_COUNT_INC; // Count up on falling
    
    left.lctrl_mode = PCNT_MODE_KEEP;
    left.hctrl_mode = PCNT_MODE_KEEP;
    left.counter_h_lim = PCNT_H_LIM;
    left.counter_l_lim = PCNT_L_LIM;

    pcnt_unit_config(&left);

    // Filter: 100 cycles at 80MHz is ~1.25us. 
    // If you have a noisy mechanical switch, increase to 1000. 
    // If optical, 100 is fine.
    pcnt_set_filter_value(PCNT_LEFT_UNIT, 150); 
    pcnt_filter_enable(PCNT_LEFT_UNIT);

    // Clear and Start
    pcnt_counter_pause(PCNT_LEFT_UNIT);
    pcnt_counter_clear(PCNT_LEFT_UNIT);
    pcnt_counter_resume(PCNT_LEFT_UNIT);


    // --- RIGHT UNIT SETUP ---
    pcnt_config_t right = left; // Copy config
    right.unit = PCNT_RIGHT_UNIT;
    right.pulse_gpio_num = ENCODER_B_1; // Assign Right Pin

    pcnt_unit_config(&right);
    
    pcnt_set_filter_value(PCNT_RIGHT_UNIT, 150);
    pcnt_filter_enable(PCNT_RIGHT_UNIT);
    
    pcnt_counter_pause(PCNT_RIGHT_UNIT);
    pcnt_counter_clear(PCNT_RIGHT_UNIT);
    pcnt_counter_resume(PCNT_RIGHT_UNIT);

    // Take initial snapshot
    pcnt_get_counter_value(PCNT_LEFT_UNIT, &left_count_snapshot);
    pcnt_get_counter_value(PCNT_RIGHT_UNIT, &right_count_snapshot);

    Serial.println("PCNT encoders initialized (Double Edge Mode).");
}




void DiffCar::velocity_update() {

    unsigned long now_us = micros();
    const unsigned long window_us = 50000UL; // 50 ms
    if (now_us - last_vel_update_us < window_us) return;
    unsigned long dt = now_us - last_vel_update_us;
    last_vel_update_us = now_us;

    int16_t left_now = 0;
    int16_t right_now = 0;
    
    pcnt_get_counter_value(PCNT_LEFT_UNIT,  &left_now);
    pcnt_get_counter_value(PCNT_RIGHT_UNIT, &right_now);

    int16_t dleft = left_now - left_count_snapshot;
    int16_t dright = right_now - right_count_snapshot;

    left_count_snapshot = left_now;
    right_count_snapshot = right_now;

    float left_freq  = (float)dleft  * 1e6f / dt;
    float right_freq = (float)dright * 1e6f / dt;

    // EMA filter
    const float alpha = 0.4f;

    left_freq_filtered  = (1 - alpha) * left_freq_filtered  + alpha * left_freq;
    right_freq_filtered = (1 - alpha) * right_freq_filtered + alpha * right_freq;

    if (left_freq_filtered < MIN_PULSES) left_freq_filtered = 0;
    if (right_freq_filtered < MIN_PULSES) right_freq_filtered = 0;

    left_velocity_ms  = left_freq_filtered  * METER_PER_PULSE;
    right_velocity_ms = right_freq_filtered * METER_PER_PULSE;
}


void DiffCar::debug_encoder() {
      Serial.print("encoder Left: "); Serial.println(diffCar.left_count_snapshot);
      Serial.print("LV (m/s): ");
      Serial.print(left_velocity_ms, 2);
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