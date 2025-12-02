// Include DiffCar class definition
#include "diff_car.h"  
#include <cmath>
#include <QuickPID.h>
constexpr float PID_DT_S = 1.0f / 100.0f; // handler_motor roda ~100 Hz
float Kp_p = 2, Ki_p = 1, Kd_p = 0.1;

QuickPID left_pid(&diffCar.left_velocity_ms, &diffCar.left_pid_output, &diffCar.abs_left_velocity_target, Kp_p, Ki_p, Kd_p,  /* OPTIONS */
               left_pid.pMode::pOnError,                   /* pOnError, pOnMeas, pOnErrorMeas */
               left_pid.dMode::dOnMeas,                    /* dOnError, dOnMeas */
               left_pid.iAwMode::iAwCondition,             /* iAwCondition, iAwClamp, iAwOff */
               left_pid.Action::direct);  
QuickPID right_pid(&diffCar.left_velocity_ms, &diffCar.right_pid_output, &diffCar.abs_right_velocity_target, Kp_p, Ki_p, Kd_p,  /* OPTIONS */
               right_pid.pMode::pOnError,                   /* pOnError, pOnMeas, pOnErrorMeas */
               right_pid.dMode::dOnMeas,                    /* dOnError, dOnMeas */
               right_pid.iAwMode::iAwCondition,             /* iAwCondition, iAwClamp, iAwOff */
               right_pid.Action::direct);

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

    //turn the PID on
    left_pid.SetMode(left_pid.Control::automatic);
    left_pid.SetOutputLimits(-1, 1);
    left_pid.SetSampleTimeUs(2000);
        
    right_pid.SetMode(right_pid.Control::automatic);
    right_pid.SetOutputLimits(-1, 1);
    right_pid.SetSampleTimeUs(2000);
 
}

void DiffCar::set_motor_speed(float left_motor_pwm_in, float right_motor_pwm_in){ 
    left_motor_pwm = constrain(fabs(left_motor_pwm_in), 0.0f, 255.0f);
    right_motor_pwm = constrain(fabs(right_motor_pwm_in), 0.0f, 255.0f);
}

static inline double model_velocity_from_pwm(double p){
    return (C1 * p * p) + (C2 * p) + C3;
}

static double pwm_from_velocity(double vel) {
    
    if (!std::isfinite(vel) || vel <= 0.0) return 0.0;
    const double vel_max = model_velocity_from_pwm(255.0);
    if (vel >= vel_max) return 255.0;
    const double MIN_EFFECTIVE_VEL = 0.10; 
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

void DiffCar::handler_motor(){
    abs_left_velocity_target = fabs(left_velocity_target);
    abs_right_velocity_target = fabs(right_velocity_target);

    float vel_left = this->left_velocity_target*(1+this->left_pid_output);
    float vel_right = this->right_velocity_target*(1+this->right_pid_output);

    MotorPwmResult temp_pwm = set_motor_speed_msr(vel_left, vel_right);

    float temp_pwm_left  =  temp_pwm.left;

    float temp_pwm_right = temp_pwm.right;

    if (left_velocity_target != 0.0f){
      left_pid.Compute();  
    }else{
      left_pid.Reset();
      left_pid_output = 0.0f;
      temp_pwm_left = 0.0f;
    }
    
    if (right_velocity_target != 0.0f){
        right_pid.Compute();
    }else{
        right_pid.Reset();
        right_pid_output = 0.0f;
        temp_pwm_right = 0.0f;
    }

    if(left_velocity_target < 0){
        left_motor_dir = 0;
    }else{
        left_motor_dir = 1;
    }

    if(right_velocity_target < 0){
        right_motor_dir = 0;
    }else{
        right_motor_dir = 1;
    }

    set_motor_speed(temp_pwm_left, temp_pwm_right);

    update_h_bridge();
}