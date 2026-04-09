// Include DiffCar class definition
#include "diff_car.h"  
#include <cmath>
#include "FuzzyControl.h"

FuzzyController left_fuzzy(-1.0, 1.0, &diffCar.abs_left_velocity_target, &diffCar.left_velocity_ms, &diffCar.left_pid_output);
FuzzyController right_fuzzy(-1.0, 1.0, &diffCar.abs_right_velocity_target, &diffCar.right_velocity_ms, &diffCar.right_pid_output);

void DiffCar::setup_h_bridge(){
   
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    //Motor A
    ledcSetup(0, 30000, 8);
    ledcAttachPin(MOTOR_IN1, 0);
    ledcSetup(1, 30000, 8);
    ledcAttachPin(MOTOR_IN2, 1);
    //Motor B
    ledcSetup(2, 30000, 8);
    ledcAttachPin(MOTOR_IN3, 2);
    ledcSetup(3, 30000, 8);
    ledcAttachPin(MOTOR_IN4, 3);

    Preferences preferences;
    preferences.begin("motor_calib", false);
    min_pwm_left = preferences.getFloat("min_pwml", 0.0f);
    min_pwm_right = preferences.getFloat("min_pwmr", 0.0f);
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
    min_pwm_left = 255.0f; // Default alto
    while(test_pwm <= 255.0f) {
        left_motor_pwm = test_pwm;
        right_motor_pwm = 0;
        update_h_bridge();
        delay(50); 
        velocity_update(); 
        
        if (fabs(left_velocity_ms) > 0.01f) {
            min_pwm_left = test_pwm + 2.0f; // Margem de seguranca
            Serial.printf("Motor Esquerdo se moveu com PWM: %.1f\n", test_pwm);
            break;
        }
        test_pwm += 1.0f;
    }
    
    // Para esquerda
    left_motor_pwm = 0;
    update_h_bridge();
    delay(1000); 

    // Mapeamento Motor Direito
    test_pwm = 0.0f;
    min_pwm_right = 255.0f;
    while(test_pwm <= 255.0f) {
        left_motor_pwm = 0;
        right_motor_pwm = test_pwm;
        update_h_bridge();
        delay(50);
        velocity_update();
        
        if (fabs(right_velocity_ms) > 0.01f) {
            min_pwm_right = test_pwm + 2.0f; // Margem de seguranca
            Serial.printf("Motor Direito se moveu com PWM: %.1f\n", test_pwm);
            break;
        }
        test_pwm += 1.0f;
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
        left_motor_pwm = min_pwm_left + (left_in / 255.0f) * (255.0f - min_pwm_left);
        left_motor_pwm = constrain(left_motor_pwm, 0.0f, 255.0f);
    } else {
        left_motor_pwm = 0.0f;
    }

    float right_in = fabs(right_motor_pwm_in);
    if (right_in > 0.01f) {
        // Mapeia gradualmente a entrada para a zona util
        right_motor_pwm = min_pwm_right + (right_in / 255.0f) * (255.0f - min_pwm_right);
        right_motor_pwm = constrain(right_motor_pwm, 0.0f, 255.0f);
    } else {
        right_motor_pwm = 0.0f;
    }
}

static inline double model_velocity_from_pwm(double p){
    return (C1 * p * p) + (C2 * p) + C3;
}

static double pwm_from_velocity(double vel) {
    
    if (!std::isfinite(vel) || vel <= 0.0) return 0.0;
    const double vel_max = model_velocity_from_pwm(255.0);
    if (vel >= vel_max) return 255.0;
    const double MIN_EFFECTIVE_VEL = 0.02; // Permite velocidades radiais baixas sem retornar 0.0
    if (vel < MIN_EFFECTIVE_VEL) return 0.0;
    const double a = C1;
    const double b = C2;
    const double c = (C3 - vel);
    if (std::fabs(a) < 1e-9) { 
        double p = (vel - C3) / b;
        if (!std::isfinite(p)) return 0.0;
        if (p < 0) p = 0; else if (p > 255) p = 255;
        return p;
    }
    double disc = b*b - 4*a*c;
    if (disc < 0) {
        return 0.0;
    }
    double sqrt_disc = std::sqrt(disc);
    double denom = 2*a;
    double r1 = (-b + sqrt_disc) / denom;
    double r2 = (-b - sqrt_disc) / denom;
    double best = -1;
    double best_err = 1e12;
    auto try_root = [&](double r){
        if (!std::isfinite(r)) return; 
        if (r < 0 || r > 255) return; 
        double v_pred = model_velocity_from_pwm(r);
        double err = std::fabs(v_pred - vel);
        if (err < best_err) { best_err = err; best = r; }
    };
    try_root(r1);
    try_root(r2);
    if (best < 0) {
        if (r1 >= 0 && r1 < r2) best = r1; else best = r2;
        if (best < 0) best = 0; if (best > 255) best = 255;
    }
    return best;
}

MotorPwmResult DiffCar::set_motor_speed_msr(float vel_left, float vel_right){
    MotorPwmResult result;
    const float MAX_REASONABLE_VEL = 3.0f;
    if (fabs(vel_left) > MAX_REASONABLE_VEL || fabs(vel_right) > MAX_REASONABLE_VEL) {
        Serial.println("[WARN] Valores passados a set_motor_speed_ms parecem PWM. Use set_motor_speed().");
        left_motor_pwm = constrain(fabs(vel_left), 0.0f, 255.0f);
        right_motor_pwm = constrain(fabs(vel_right), 0.0f, 255.0f);
        left_motor_dir = vel_left >= 0 ? 1 : 0;
        right_motor_dir = vel_right >= 0 ? 1 : 0;
        result.left = left_motor_pwm;
        result.right = right_motor_pwm;
        return result;
    }
    double p_left = pwm_from_velocity(fabs(vel_left));
    double p_right = pwm_from_velocity(fabs(vel_right));
    if (!std::isfinite(p_left)) p_left = 0; if (!std::isfinite(p_right)) p_right = 0;
    if (p_left < 0) p_left = 0; else if (p_left > 255) p_left = 255;
    if (p_right < 0) p_right = 0; else if (p_right > 255) p_right = 255;

    // Sincronização do Modelo de PWM preditivo à Inércia (Min PWM) já calibrada
    if (p_left > 0.01) p_left = min_pwm_left + (p_left / 255.0) * (255.0 - min_pwm_left);
    if (p_right > 0.01) p_right = min_pwm_right + (p_right / 255.0) * (255.0 - min_pwm_right);

    left_motor_pwm = (int)lround(p_left);
    right_motor_pwm = (int)lround(p_right);
    left_motor_dir = vel_left >= 0 ? 1 : 0;
    right_motor_dir = vel_right >= 0 ? 1 : 0;
    result.left = left_motor_pwm;
    result.right = right_motor_pwm;
    return result;
}

void DiffCar::update_h_bridge(){
    if(left_motor_pwm != 0){
        if (left_motor_dir) {
            ledcWrite(1, left_motor_pwm);
            ledcWrite(0, 0);
            
        } else {
            ledcWrite(1, 0);
            ledcWrite(0, left_motor_pwm);
        }
    } else {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }

    if(right_motor_pwm != 0){
        if (right_motor_dir) {
            ledcWrite(3, right_motor_pwm);
            ledcWrite(2, 0);
            
        } else {
            ledcWrite(3, 0);
            ledcWrite(2, right_motor_pwm);
        }
    } else {
        ledcWrite(2, 0);
        ledcWrite(3, 0);
    }
}

void DiffCar::handler_motor() {
    abs_left_velocity_target = fabs(left_velocity_target);
    abs_right_velocity_target = fabs(right_velocity_target);

    // 1. O Fuzzy deve calcular baseado no erro atual ANTES de aplicar
    if (left_velocity_target != 0.0f) {
        left_fuzzy.compute();  
    } else {
        left_fuzzy.reset();
        left_pid_output = 0.0f;
    }
    
    if (right_velocity_target != 0.0f) {
        right_fuzzy.compute();
    } else {
        right_fuzzy.reset();
        right_pid_output = 0.0f;
    }

    // 2. Aplica o fator de correção
    float raw_vel_left = this->left_velocity_target * (1 + this->left_pid_output);
    float raw_vel_right = this->right_velocity_target * (1 + this->right_pid_output);

    // =========================================================================
    // TCS (Traction Control System) com Slew Rate e telemetria da IMU
    // =========================================================================
    static float safe_vel_left = 0.0f;
    static float safe_vel_right = 0.0f;
    static float prev_enc_vel_l = 0.0f;
    static float prev_enc_vel_r = 0.0f;
    static float whl_accel_l = 0.0f;
    static float whl_accel_r = 0.0f;

    // Aceleração rotacional atual da roda (dv/dt), handler roda a aprox 10ms (0.01s)
    float raw_whl_accel_l = fabs(left_velocity_ms - prev_enc_vel_l) / 0.01f;
    float raw_whl_accel_r = fabs(right_velocity_ms - prev_enc_vel_r) / 0.01f;
    
    // Low-pass filter (Hysteresis) para evitar acionamentos falsos por frames de ruído
    whl_accel_l = 0.2f * raw_whl_accel_l + 0.8f * whl_accel_l;
    whl_accel_r = 0.2f * raw_whl_accel_r + 0.8f * whl_accel_r;

    prev_enc_vel_l = left_velocity_ms;
    prev_enc_vel_r = right_velocity_ms;

    // Aceleração linear real do chassi sentida pela IMU (módulo no plano X-Y em m/s^2)
    float imu_accel_xy = sqrt(pow(accel_d[0], 2) + pow(accel_d[1], 2)) * 9.81f;

    float max_accel_step = 0.04f; // Aceleração agressiva por padrão

    // Se a roda sofreu uma forte aceleração angular (> 5.0 m/s^2) 
    // mas a IMU diz que o chassi físico mal andou (< 0.8 m/s^2)...
    // A roda perdeu tração (Slip). Ativamos a intervenção!
    if ((whl_accel_l > 5.0f || whl_accel_r > 5.0f) && imu_accel_xy < 0.8f) {
        max_accel_step = 0.005f; // Corta o "gás" drasticamente pra roda grudar de novo
    }

    // Rampa suave para motor esquerdo
    if (raw_vel_left > safe_vel_left + max_accel_step) safe_vel_left += max_accel_step;
    else if (raw_vel_left < safe_vel_left - max_accel_step) safe_vel_left -= max_accel_step;
    else safe_vel_left = raw_vel_left;

    // Rampa suave para motor direito
    if (raw_vel_right > safe_vel_right + max_accel_step) safe_vel_right += max_accel_step;
    else if (raw_vel_right < safe_vel_right - max_accel_step) safe_vel_right -= max_accel_step;
    else safe_vel_right = raw_vel_right;

    // Corta sinal fantasma quando motor deve realmente parar
    if (left_velocity_target == 0.0f && fabs(safe_vel_left) < 0.05f) safe_vel_left = 0.0f;
    if (right_velocity_target == 0.0f && fabs(safe_vel_right) < 0.05f) safe_vel_right = 0.0f;

    // 3. Converte a velocidade final desejada em PWM usando apenas o modelo preditivo
    // (Garante que set_motor_speed_msr seja quem dita o PWM e a direção internos)
    set_motor_speed_msr(safe_vel_left, safe_vel_right); 

    // 4. Atualiza a Ponte H diretamente
    update_h_bridge();
}