// Include DiffCar class definition
#include "diff_car.h"  
#include <cmath>
#include <QuickPID.h>
#include <Preferences.h>
#include <sTune.h>

#define BITS_MOTOR 10
#define MAX_PWM 1023 // 1023 para 10 bits

QuickPID left_pid(&diffCar.abs_left_velocity_cms, &diffCar.left_pid_output, &diffCar.abs_left_velocity_target);
QuickPID right_pid(&diffCar.abs_right_velocity_cms, &diffCar.right_pid_output, &diffCar.abs_right_velocity_target);

void DiffCar::setup_h_bridge(){
    Preferences prefs;
    prefs.begin("fuzzy", false);
    
    // Load PID values, but protect against corrupted NaN/Inf values from previous failed Autotune attempts
    // Valores atualizados do último Auto-Tune:
    float kp_l = prefs.getFloat("Kp_left", 8.860f);
    float ki_l = prefs.getFloat("Ki_left", 1.171f);
    float kd_l = prefs.getFloat("Kd_left", 0.0f);
    
    float kp_r = prefs.getFloat("Kp_right", 10.945f);
    float ki_r = prefs.getFloat("Ki_right", 1.157f);
    float kd_r = prefs.getFloat("Kd_right", 0.0f);
    
    if (isnan(kp_l) || isinf(kp_l)) kp_l = 8.860f;
    if (isnan(ki_l) || isinf(ki_l)) ki_l = 1.171f;
    if (isnan(kd_l) || isinf(kd_l)) kd_l = 0.0f;
    
    if (isnan(kp_r) || isinf(kp_r)) kp_r = 10.945f;
    if (isnan(ki_r) || isinf(ki_r)) ki_r = 1.157f;
    if (isnan(kd_r) || isinf(kd_r)) kd_r = 0.0f;

    left_pid.SetTunings(kp_l, ki_l, kd_l);
    right_pid.SetTunings(kp_r, ki_r, kd_r);

    prefs.end();

    left_pid.SetSampleTimeUs(LOOP_FAST_US);
    right_pid.SetSampleTimeUs(LOOP_FAST_US);
    left_pid.SetMode(QuickPID::Control::automatic);
    right_pid.SetMode(QuickPID::Control::automatic);
    left_pid.SetOutputLimits(400, MAX_PWM);
    right_pid.SetOutputLimits(400, MAX_PWM);

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

    // Se a inercia nao foi calibrada, chama a calibracao automatica
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
    // Alimenta as variáveis absolutas lidas pelos PIDs
    abs_left_velocity_target = fabs(left_velocity_target);
    abs_right_velocity_target = fabs(right_velocity_target);

    left_motor_dir = (left_velocity_target >= 0) ? 1 : 0;
    right_motor_dir = (right_velocity_target >= 0) ? 1 : 0;

    // ── Motor Esquerdo ────────────────────────────
    if (left_velocity_target != 0.0f) {
        left_pid.Compute(); // O PID faz as contas livremente, acumulando o erro
        
        if (abs_left_velocity_cms < 0.1f) {
            // Roda parada: envia o tranco de 771 diretamente para o motor para vencer a inércia
            left_motor_pwm = 1023.0f;
        } else {
            // Roda em movimento: o atrito é menor, entregamos o controlo ao PID
            left_motor_pwm = left_pid_output;
        }
    } else {
        left_pid.Reset();
        left_pid_output = 0.0f;
        left_motor_pwm = 0.0f;
    }
    
    // ── Motor Direito ────────────────────────────
    if (right_velocity_target != 0.0f) {
        right_pid.Compute();
        
        if (abs_right_velocity_cms < 0.1f) {
            right_motor_pwm = 1023.0f;
        } else {
            right_motor_pwm = right_pid_output;
        }
    } else {
        right_pid.Reset();
        right_pid_output = 0.0f;
        right_motor_pwm = 0.0f;
    }

    update_h_bridge();
}


void DiffCar::auto_tune_pid() {

    Serial.println("=== AUTO TUNE START ESQUERDO ===");

    float tune_in = 0.0f;
    float tune_out = 0.0f;

    sTune tuner(&tune_in, &tune_out, sTune::ZN_PID, sTune::directIP, sTune::printSUMMARY);

    tuner.Configure(
        150.0f,        // input span
        1023,       // output span
        0,  // start output
        1023 * 0.7f,// step
        5,            // test time
        10,             // settle
        200            // samples
    );

    tuner.SetEmergencyStop(200);

    left_motor_dir = 1;

    bool done = false;

    while (!done) {

        velocity_update();
        tune_in = left_velocity_cms;

        uint8_t status = tuner.Run();

        if (status == sTune::sample) {

            left_motor_pwm = tune_out;
            update_h_bridge();

            Serial.printf("OUT: %.1f | Vel: %.2f\n", left_motor_pwm, tune_in);

        }

        if (status == sTune::tunings) {
                left_motor_pwm = 0;
                update_h_bridge();

                Serial.println("\n================================================");
                Serial.println("  TESTE CONCLUÍDO! CALCULANDO TODAS AS REGRAS  ");
                Serial.println("================================================");

                // Imprime as sintonias PID (Proporcional, Integral, Derivativo)
                tuner.SetTuningMethod(sTune::ZN_PID);
                tuner.printTunings();
                
                tuner.SetTuningMethod(sTune::DampedOsc_PID);
                tuner.printTunings();
                
                tuner.SetTuningMethod(sTune::NoOvershoot_PID);
                tuner.printTunings();
                
                tuner.SetTuningMethod(sTune::CohenCoon_PID);
                tuner.printTunings();
                
                tuner.SetTuningMethod(sTune::Mixed_PID);
                tuner.printTunings();

                // Imprime as sintonias PI (Apenas Proporcional e Integral - Ótimo se tiver ruído)
                Serial.println("\n--- SINTONIAS APENAS PI (Sem Derivativo) ---");
                tuner.SetTuningMethod(sTune::ZN_PI);
                tuner.printTunings();

                tuner.SetTuningMethod(sTune::NoOvershoot_PI);
                tuner.printTunings();

                done = true;
        }

        delay(10); // 🔥 CRÍTICO (não usar microseconds aqui!)
    }

    left_motor_pwm = 0;
    update_h_bridge();

    float kp = tuner.GetKp();
    float ki = tuner.GetKi();
    float kd = tuner.GetKd();

    if (isnan(kp) || isinf(kp)) {
        Serial.println("❌ Tuning falhou");
        return;
    }

    Serial.printf("✅ESQUERDO Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
    
    Serial.println("=== AUTO TUNE START DIREITO ===");

    tune_in = 0.0f;
    tune_out = 0.0f;

    sTune tuner2(&tune_in, &tune_out, sTune::ZN_PID, sTune::directIP, sTune::printSUMMARY);

    tuner2.Configure(
        150.0f,        // input span
        1023,       // output span
        0,  // start output
        1023 * 0.7f,// step
        5,            // test time
        10,             // settle
        200            // samples
    );

    tuner2.SetEmergencyStop(200);

    right_motor_dir = 1;

    done = false;

    while (!done) {

        velocity_update();
        tune_in = right_velocity_cms;

        uint8_t status = tuner2.Run();

        if (status == sTune::sample) {

            right_motor_pwm = tune_out;
            update_h_bridge();

            Serial.printf("OUT: %.1f | Vel: %.2f\n", right_motor_pwm, tune_in);

        }

        if (status == sTune::tunings) {
                right_motor_pwm = 0;
                update_h_bridge();

                Serial.println("\n================================================");
                Serial.println("  TESTE CONCLUÍDO! CALCULANDO TODAS AS REGRAS  ");
                Serial.println("================================================");

                // Imprime as sintonias PID (Proporcional, Integral, Derivativo)
                tuner2.SetTuningMethod(sTune::ZN_PID);
                tuner2.printTunings();
                
                tuner2.SetTuningMethod(sTune::DampedOsc_PID);
                tuner2.printTunings();
                
                tuner2.SetTuningMethod(sTune::NoOvershoot_PID);
                tuner2.printTunings();
                
                tuner2.SetTuningMethod(sTune::CohenCoon_PID);
                tuner2.printTunings();
                
                tuner2.SetTuningMethod(sTune::Mixed_PID);
                tuner2.printTunings();

                // Imprime as sintonias PI (Apenas Proporcional e Integral - Ótimo se tiver ruído)
                Serial.println("\n--- SINTONIAS APENAS PI (Sem Derivativo) ---");
                tuner2.SetTuningMethod(sTune::ZN_PI);
                tuner2.printTunings();

                tuner2.SetTuningMethod(sTune::NoOvershoot_PI);
                tuner2.printTunings();

                done = true;
        }

        delay(10); // 🔥 CRÍTICO (não usar microseconds aqui!)
    }

    right_motor_pwm = 0;
    update_h_bridge();

    kp = tuner2.GetKp();
    ki = tuner2.GetKi();
    kd = tuner2.GetKd();

    if (isnan(kp) || isinf(kp)) {
        Serial.println("❌ Tuning falhou");
        return;
    }
    Serial.printf("✅DIREITO Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);


}
