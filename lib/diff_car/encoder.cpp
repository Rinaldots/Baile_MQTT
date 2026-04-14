#include "diff_car.h"
#include <algorithm>
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"

#define PCNT_LEFT_UNIT   PCNT_UNIT_0
#define PCNT_RIGHT_UNIT  PCNT_UNIT_1

#ifdef ENCODER_SIMPLE


volatile uint32_t last_time_left = 0;
volatile uint32_t interval_left = 0;
volatile uint32_t last_time_right = 0;
volatile uint32_t interval_right = 0;

// ISR (Interrupt Service Routine)
static void IRAM_ATTR pcnt_intr_handler(void *arg) {
    uint32_t now = micros();
    uint32_t intr_status = PCNT.int_st.val;
    
    if (intr_status & BIT(PCNT_LEFT_UNIT)) {
        uint32_t dt = now - last_time_left;
        // Debounce: ignora pulsos mais rápidos que 1ms (1000 us = max 1000 pps)
        if (dt > 1000) {
            interval_left = dt;
            last_time_left = now;
        }
        PCNT.int_clr.val = BIT(PCNT_LEFT_UNIT); // Limpa flag de interrupção
        // Reseta o contador para o próximo pulso acionar h_lim=1 novamente
        PCNT.ctrl.cnt_rst_u0 = 1;
        PCNT.ctrl.cnt_rst_u0 = 0;
    }
    if (intr_status & BIT(PCNT_RIGHT_UNIT)) {
        uint32_t dt = now - last_time_right;
        // Debounce: ignora pulsos mais rápidos que 1ms
        if (dt > 1000) {
            interval_right = dt;
            last_time_right = now;
        }
        PCNT.int_clr.val = BIT(PCNT_RIGHT_UNIT); // Limpa flag de interrupção
        // Reseta o contador
        PCNT.ctrl.cnt_rst_u1 = 1;
        PCNT.ctrl.cnt_rst_u1 = 0;
    }
}

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
    left.pos_mode  = PCNT_COUNT_INC;
    left.neg_mode  = PCNT_COUNT_DIS;

    left.lctrl_mode = PCNT_MODE_KEEP;
    left.hctrl_mode = PCNT_MODE_KEEP;
    left.counter_h_lim = 1; 
    left.counter_l_lim = 0;
    pcnt_unit_config(&left);

    // Filtro APB clock cycles (80MHz -> 1 clk = 12.5ns)
    // 1023 clks = ~12.7us (ignora pulsos muito curtos/ruído HF típico de opto)
    pcnt_set_filter_value(PCNT_LEFT_UNIT, 1023);
    pcnt_filter_enable(PCNT_LEFT_UNIT);

    // Habilita interrupção quando atingir o limite superior (Thres0 ou H_Lim)
    pcnt_event_enable(PCNT_LEFT_UNIT, PCNT_EVT_H_LIM);
    
    // Configura o Right igual
    pcnt_config_t right = left;
    right.unit = PCNT_RIGHT_UNIT;
    right.pulse_gpio_num = ENCODER_B_1;
    pcnt_unit_config(&right);
    pcnt_set_filter_value(PCNT_RIGHT_UNIT, 1023);
    pcnt_filter_enable(PCNT_RIGHT_UNIT);
    pcnt_event_enable(PCNT_RIGHT_UNIT, PCNT_EVT_H_LIM);

    // Registra o handler global de interrupções do PCNT
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_LEFT_UNIT);
    pcnt_intr_enable(PCNT_RIGHT_UNIT);


    
    pcnt_counter_pause(PCNT_LEFT_UNIT);
    pcnt_counter_clear(PCNT_LEFT_UNIT);
    pcnt_counter_resume(PCNT_LEFT_UNIT);

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
    const float Q = 5.0f;   // aceita mudanças rápidas de velocidade
    const float R = 1500.0f;   // chave óptica tem pouco ruído (R/Q = 3x)

    float P_pred = P + Q;
    float K      = P_pred / (P_pred + R);

    x = x + K * (measurement - x);
    P = (1.0f - K) * P_pred;

    return x;
}

void DiffCar::velocity_update() {
    unsigned long now_us = micros();
    if (now_us - last_vel_update_us < LOOP_FAST_US) return;
    last_vel_update_us = now_us;

    // Proteção para leitura das variáveis voláteis
    noInterrupts();
    uint32_t dt_l = interval_left;
    uint32_t dt_r = interval_right;
    uint32_t last_l = last_time_left;
    uint32_t last_r = last_time_right;
    interrupts();

    float left_freq = 0;
    uint32_t current_dt_l = now_us - last_l;
    // Timeout: se não houver pulso há mais de 100ms, freq = 0
    if (dt_l > 0 && current_dt_l < 100000) {
        // Se já passou mais tempo desde o último pulso do que o intervalo anterior (desacelerando)
        if (current_dt_l > dt_l) {
            left_freq = 1000000.0f / current_dt_l;
        } else {
            left_freq = 1000000.0f / dt_l;
        }
    }

    float right_freq = 0;
    uint32_t current_dt_r = now_us - last_r;
    if (dt_r > 0 && current_dt_r < 100000) {
        if (current_dt_r > dt_r) {
            right_freq = 1000000.0f / current_dt_r;
        } else {
            right_freq = 1000000.0f / dt_r;
        }
    }

    // Aplica o Kalman e o sinal de direção como você já fazia
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
      
      Serial.print(" | LV(cm/s): ");
      Serial.print(left_velocity_cms, 2);
      Serial.print(" | TLV: ");
      Serial.print(left_velocity_target, 2);
      Serial.print(" | Lp: ");
      Serial.println(left_motor_pwm, 2);
    
        Serial.print(" | RV(cm/s): ");
        Serial.print(right_velocity_cms, 2);
        Serial.print(" | TRV: ");
        Serial.print(right_velocity_target, 2);
        Serial.print(" | Rp: ");
        Serial.println(right_motor_pwm, 2);
}


void DiffCar::debug_raw_pins() {
    Serial.print("Raw A: ");
    Serial.print(digitalRead(ENCODER_A_1));
    Serial.print(" | Raw B: ");
    Serial.println(digitalRead(ENCODER_B_1));
}

#endif
