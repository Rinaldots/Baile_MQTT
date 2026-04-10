// Include DiffCar class definition
#include "diff_car.h"  
#include <cmath>
#include <QuickPID.h>
#include <Preferences.h>

#define BITS_MOTOR 10
#define MAX_PWM (pow(2, BITS_MOTOR) - 1) // 65535 para 16 bits

QuickPID left_pid(&diffCar.left_velocity_cms, &diffCar.left_pid_output, &diffCar.abs_left_velocity_target);
QuickPID right_pid(&diffCar.right_velocity_cms, &diffCar.right_pid_output, &diffCar.abs_right_velocity_target);

void DiffCar::setup_h_bridge(){
    Preferences prefs;
    prefs.begin("fuzzy", true);
    left_pid.SetTunings(prefs.getFloat("Kp_left", 50.0f), prefs.getFloat("Ki_left", 1.0f), prefs.getFloat("Kd_left", 1.0f));
    right_pid.SetTunings(prefs.getFloat("Kp_right", 50.0f), prefs.getFloat("Ki_right", 1.0f), prefs.getFloat("Kd_right", 1.0f));

    prefs.end();
    left_pid.SetSampleTimeUs(LOOP_FAST_US);
    right_pid.SetSampleTimeUs(LOOP_FAST_US);
    left_pid.SetMode(left_pid.Control::automatic);
    right_pid.SetMode(right_pid.Control::automatic);

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    //Motor A
    ledcSetup(0, 1000, BITS_MOTOR);
    ledcAttachPin(MOTOR_IN1, 0);
    ledcSetup(1, 1000, BITS_MOTOR);
    ledcAttachPin(MOTOR_IN2, 1);
    //Motor B
    ledcSetup(2, 1000, BITS_MOTOR);
    ledcAttachPin(MOTOR_IN3, 2);
    ledcSetup(3, 1000, BITS_MOTOR);
    ledcAttachPin(MOTOR_IN4, 3);

    Preferences preferences;
    preferences.begin("motor_calib", false);
    min_pwm_left = preferences.getFloat("min_pwml", 0.0f);
    min_pwm_right = preferences.getFloat("min_pwmr", 0.0f);
    left_pid.SetOutputLimits(min_pwm_left, MAX_PWM);
    right_pid.SetOutputLimits(min_pwm_right, MAX_PWM);
    preferences.end();
    
    // Se a inercia nao foi calibrada, chama a calibracao auto
    if (min_pwm_left == 0.0f || min_pwm_right == 0.0f) {
        calibrate_motors_inertia();
    } else {
        Serial.printf("Valores PWM de Inercia carregados: L=%.1f R=%.1f\n", min_pwm_left, min_pwm_right);
    }
}

void DiffCar::calibrate_motors_inertia() {
    Serial.println("==============================================");
    Serial.println(" Iniciando Mapeamento Automatico de Inercia ");
    Serial.println(" (Motor girara devagar para encontrar o limite) ");
    Serial.println("==============================================");
    
    left_motor_dir = 1;
    right_motor_dir = 1;
    
    // Mapeamento Motor Esquerdo
    float test_pwm = 0.0f;
    min_pwm_left = MAX_PWM; // Default alto
    while(test_pwm <= MAX_PWM) {
        left_motor_pwm = test_pwm;
        right_motor_pwm = 0;
        update_h_bridge();
        delay(50); 
        velocity_update(); 
        
        if (fabs(left_velocity_cms) > 0.01f) {
            min_pwm_left = test_pwm + MAX_PWM/100; // Margem de seguranca
            Serial.printf("Motor Esquerdo se moveu com PWM: %.1f\n", test_pwm);
            break;
        }
        test_pwm += MAX_PWM/100;
    }
    
    // Para esquerda
    left_motor_pwm = 0;
    update_h_bridge();
    delay(1000); 

    // Mapeamento Motor Direito
    test_pwm = 0.0f;
    min_pwm_right = MAX_PWM;
    while(test_pwm <= MAX_PWM) {
        left_motor_pwm = 0;
        right_motor_pwm = test_pwm;
        update_h_bridge();
        delay(50);
        velocity_update();
        
        if (fabs(right_velocity_cms) > 0.01f) {
            min_pwm_right = test_pwm + MAX_PWM/100; // Margem de seguranca
            Serial.printf("Motor Direito se moveu com PWM: %.1f\n", test_pwm);
            break;
        }
        test_pwm += MAX_PWM/100;
    }
    
    // Para direita
    right_motor_pwm = 0;
    update_h_bridge();

    // Salvar na NVS (EEPROM Emulada do ESP32)
    Preferences preferences;
    preferences.begin("motor_calib", false);
    preferences.putFloat("min_pwml", min_pwm_left);
    preferences.putFloat("min_pwmr", min_pwm_right);
    preferences.end();
    
    Serial.printf("Calibracao Finalizada com Sucesso L: %.1f, R: %.1f\n", min_pwm_left, min_pwm_right);
}

void DiffCar::set_motor_speed(float left_motor_pwm_in, float right_motor_pwm_in){ 
    float left_in = fabs(left_motor_pwm_in);
    if (left_in > 0.01f) {
        // Mapeia gradualmente a entrada para a zona util
        left_motor_pwm = min_pwm_left + (left_in / MAX_PWM) * (MAX_PWM - min_pwm_left);
        left_motor_pwm = constrain(left_motor_pwm, 0.0f, MAX_PWM);
    } else {
        left_motor_pwm = 0.0f;
    }

    float right_in = fabs(right_motor_pwm_in);
    if (right_in > 0.01f) {
        // Mapeia gradualmente a entrada para a zona util
        right_motor_pwm = min_pwm_right + (right_in / MAX_PWM) * (MAX_PWM - min_pwm_right);
        right_motor_pwm = constrain(right_motor_pwm, 0.0f, MAX_PWM);
    } else {
        right_motor_pwm = 0.0f;
    }
}

void DiffCar::update_h_bridge(){
    if(left_motor_pwm != 0){
        if (left_motor_dir) {            
            ledcWrite(1, left_motor_pwm);            
            ledcWrite(0, 0);} else {
            ledcWrite(1, 0);            
            ledcWrite(0, left_motor_pwm);}} else {ledcWrite(0, 0);        
                ledcWrite(1, 0);}if(right_motor_pwm != 0){if (right_motor_dir) {            
                ledcWrite(3, right_motor_pwm);            
                ledcWrite(2, 0);} else {ledcWrite(3, 0);            
                ledcWrite(2, right_motor_pwm);}} else {        
                    ledcWrite(2, 0);        
                    ledcWrite(3, 0);}
}

void DiffCar::handler_motor() {
    abs_left_velocity_target = fabs(left_velocity_target);
    abs_right_velocity_target = fabs(right_velocity_target);

    // ── Fuzzy computa PWM diretamente ────────────────────────────
    if (left_velocity_target != 0.0f) {
        left_pid.Compute();  
    } else {
        left_pid.Reset();
        left_pid_output = 0.0f;
    }
    
    if (right_velocity_target != 0.0f) {
        right_pid.Compute();
    } else {
        right_pid.Reset();
        right_pid_output = 0.0f;
    }

    // ── TCS — igual ao original ───────────────────────────────────
    static float safe_pwm_left  = 0.0f;
    static float safe_pwm_right = 0.0f;
    static float prev_enc_vel_l = 0.0f;
    static float prev_enc_vel_r = 0.0f;
    static float whl_accel_l    = 0.0f;
    static float whl_accel_r    = 0.0f;

    float raw_whl_accel_l = fabs(left_velocity_cms  - prev_enc_vel_l) / 0.01f;
    float raw_whl_accel_r = fabs(right_velocity_cms - prev_enc_vel_r) / 0.01f;
    
    whl_accel_l = 0.2f * raw_whl_accel_l + 0.8f * whl_accel_l;
    whl_accel_r = 0.2f * raw_whl_accel_r + 0.8f * whl_accel_r;

    prev_enc_vel_l = left_velocity_cms;
    prev_enc_vel_r = right_velocity_cms;

    float imu_accel_xy = sqrtf(powf(accel_d[0], 2) + powf(accel_d[1], 2)) * 9.81f;

    // Slew rate em PWM (era 0.02 de vel → ~5 PWM equivalente)
    float max_step  = 5.0f;
    float slip_step = 0.5f;

    if ((whl_accel_l > 3.0f || whl_accel_r > 3.0f) && imu_accel_xy < 0.5f) {
        max_step = slip_step;
    }

    // ── Rampa sobre o PWM fuzzy ───────────────────────────────────
    float raw_pwm_left  = left_pid_output;   // já é PWM (0–1023)
    float raw_pwm_right = right_pid_output;

    if      (raw_pwm_left  > safe_pwm_left  + max_step) safe_pwm_left  += max_step;
    else if (raw_pwm_left  < safe_pwm_left  - max_step) safe_pwm_left  -= max_step;
    else safe_pwm_left  = raw_pwm_left;

    if      (raw_pwm_right > safe_pwm_right + max_step) safe_pwm_right += max_step;
    else if (raw_pwm_right < safe_pwm_right - max_step) safe_pwm_right -= max_step;
    else safe_pwm_right = raw_pwm_right;

    if (left_velocity_target  == 0.0f && safe_pwm_left  < 3.0f) safe_pwm_left  = 0.0f;
    if (right_velocity_target == 0.0f && safe_pwm_right < 3.0f) safe_pwm_right = 0.0f;

    // ── Aplica direção e min_pwm de inércia ──────────────────────
    left_motor_dir  = (left_velocity_target  >= 0.0f) ? 1 : 0;
    right_motor_dir = (right_velocity_target >= 0.0f) ? 1 : 0;

    // Compensa inércia: mapeia PWM fuzzy (0–1023) para zona útil real
    auto apply_inertia = [](float pwm, float min_pwm) -> int {
        if (pwm < 1.0f) return 0;
        float mapped = min_pwm + (pwm / MAX_PWM) * (MAX_PWM - min_pwm);
        return (int)constrain(mapped, 0.0f, MAX_PWM);
    };

    left_motor_pwm  = apply_inertia(safe_pwm_left,  min_pwm_left);
    right_motor_pwm = apply_inertia(safe_pwm_right, min_pwm_right);

    // 4. Atualiza a Ponte H diretamente
    update_h_bridge();
}
