
#include "diff_car.h"

#include <stdio.h>
#include <math.h>
#include <tinyekf.h> 
#include <math.h>

static ekf_t ekf; 

static const double Q[EKF_N * EKF_N] = {
    1e-4, 0,    0,     0,      0,
    0,    1e-4, 0,     0,      0,
    0,    0,    1e-3,  0,      0,
    0,    0,    0,     2e-2,   0,
    0,    0,    0,     0,      5e-5   // <<< AQUI! antes era 1e-6
};

static double R[EKF_M * EKF_M] = {
    0.005, 0,       // velocidade do encoder (suficiente)
    0,     0.002     // yaw: agora ele confia mais nas medições!
};



DiffCar::DiffCar() {}

static double wrapAngleD(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

void DiffCar::setup() {
  setup_h_bridge();
  setup_encoder();
  setup_mpu();
  setup_leds();
  update_leds(0,1,0,1,0,1,0,1);
}

void DiffCar::encoder_odometry_update() {

    float vl = left_velocity_ms;
    float vr = right_velocity_ms; 
    if (!left_motor_dir){
        vl = -vl;
    }
    if (!right_motor_dir){
        vr = -vr;
    }
    float L = WHEEL_BASE_M;     
    float v = (vr + vl) / 2.0f;             
    float omega = (vr - vl) / L;              
    odom_enc.linear_velocity = v;
    odom_enc.angular_velocity = omega;
}

void DiffCar::ekf_init_diffcar()
{
    const double Pdiag[EKF_N] = {0.1, 0.1, 0.1, 0.1, 0.01};
    ekf_initialize(&ekf, Pdiag);
}

void DiffCar::ekf_step_with_gyro_rate(float dt, double delta_gyro, float v_enc, double yaw_meas_rad)
{   
    // ----------------------------------------------------
    // Declara apenas UMA vez
    // ----------------------------------------------------
    double z[EKF_M];
    double hx[EKF_M];
    double Hjac[EKF_M * EKF_N];  // renomeado para evitar conflito com o H antigo
                                 // (opcional, só p/ limpeza)

    bool valid_encoder = true;
    float max_phys_speed = 1.0f; // m/s
    if (fabs(v_enc) > max_phys_speed) valid_encoder = false;

    if (!valid_encoder) {
        R[0*EKF_M + 0] = 1.0;
    } else {
        R[0*EKF_M + 0] = 0.005;
    }

    if (dt <= 0.0) return;

    bool is_stationary = false;
    float gyro_mag = fabs(diffCar.gyro_d[2]);
    float acc_dev = sqrt(diffCar.accel_d[0]*diffCar.accel_d[0] + diffCar.accel_d[1]*diffCar.accel_d[1] + diffCar.accel_d[2]*diffCar.accel_d[2]);
    if (gyro_mag < 0.02f && acc_dev < 0.2f && odom_enc.linear_velocity == 0 && odom_enc.angular_velocity == 0) {
        is_stationary = true;
        //Serial.println("EKF: Robot is stationary.");  
    }

    if (is_stationary) {
        ekf.x[3] = 0.0;  // v = 0
        ekf.x[4] = ekf.x[4]; // mantém bias
        ekf.x[2] = yaw_meas_rad; // trava yaw no IMU

        // P reduzido → confiança alta
        for (int i = 0; i < EKF_N; i++) {
            ekf.P[i * EKF_N + i] *= 0.1;  // acessa diagonal
        }
        return;
    }

    // ----------------------------------------------------
    // 1) Modelo de processo
    // ----------------------------------------------------
    double fx[EKF_N];
    fx[0] = ekf.x[0] + ekf.x[3] * cos(ekf.x[2]) * dt;
    fx[1] = ekf.x[1] + ekf.x[3] * sin(ekf.x[2]) * dt;
    fx[2] = ekf.x[2] + (delta_gyro - ekf.x[4]) * dt;
    fx[3] = ekf.x[3];
    fx[4] = ekf.x[4];

    fx[2] = wrapAngleD(fx[2]);

    // ----------------------------------------------------
    // 2) Jacobiana F
    // ----------------------------------------------------
    double F[EKF_N * EKF_N] = {0};

    for (int i = 0; i < EKF_N; i++) F[i*EKF_N + i] = 1.0;

    double theta = fx[2];
    double v = fx[3];

    F[0*EKF_N + 2] = -v * sin(theta) * dt;
    F[0*EKF_N + 3] =  cos(theta) * dt;
    F[1*EKF_N + 2] =  v * cos(theta) * dt;
    F[1*EKF_N + 3] =  sin(theta) * dt;
    F[2*EKF_N + 4] = -(double)dt;

    ekf_predict(&ekf, fx, F, Q);

    // ----------------------------------------------------
    // 3) Medições Z (somente UMA vez!)
    // ----------------------------------------------------
    z[0] = (double)v_enc;

    double yaw_err = wrapAngleD(yaw_meas_rad - ekf.x[2]);
    z[1] = wrapAngleD(ekf.x[2] + yaw_err);

    // ----------------------------------------------------
    // 4) hx e H
    // ----------------------------------------------------
    hx[0] = ekf.x[3];
    hx[1] = ekf.x[2];

    for (int i=0;i<EKF_M*EKF_N;i++) Hjac[i] = 0.0;
    Hjac[0*EKF_N + 3] = 1.0;
    Hjac[1*EKF_N + 2] = 1.0;

    // ----------------------------------------------------
    // 5) EKF Update
    // ----------------------------------------------------
    ekf_update(&ekf, z, hx, Hjac, R);

    ekf.x[2] = wrapAngleD(ekf.x[2]);

    // ----------------------------------------------------
    // 6) Exporta odometria
    // ----------------------------------------------------
    odom_real.x = (float)ekf.x[0];
    odom_real.y = (float)ekf.x[1];
    odom_real.theta = (float)ekf.x[2];
    odom_real.linear_velocity = (float)ekf.x[3];
    odom_real.angular_velocity = (float)(delta_gyro - ekf.x[4]);
}


void DiffCar::navigate_delta(float delta_x, float delta_y, float delta_theta) 
{
    this->target_x = odom_real.x + delta_x;
    this->target_theta = odom_real.theta + delta_theta;
}

void DiffCar::navigate_to(float target_x, float target_y, float target_theta,
                          float precision, float target_velocity)
{

    float dx = target_x - odom_real.x;
    float distance_x = fabs(dx);

    float angle_error = target_theta - odom_real.theta;

    float current_w = odom_enc.angular_velocity; 

    if (distance_x < precision)
    {
        float Kp_rot = 0.05f; 
        float Kd_rot = 0.2f; 
        float w = (Kp_rot * angle_error) - (Kd_rot * current_w);
        if (fabs(angle_error) > 0.04f && fabs(w) < VEL_MIN_ANG)
            w = copysign(VEL_MIN_ANG, w);
        w = constrain(w, -VEL_MAX_ANG, VEL_MAX_ANG);
        float L = WHEEL_BASE_M;
        left_velocity_target  = -w * L * 0.5f;
        right_velocity_target =  w * L * 0.5f;
        if (fabs(angle_error) < 0.2f) {
            left_velocity_target = 0.0f;
            right_velocity_target = 0.0f;
            diffCar.mode = 0; 
        }
        return;
    }
    float v_cmd = target_velocity * tanh(dx * 1.5f); // 1.5 ganho de aproximação
    
    v_cmd = constrain(v_cmd, -VEL_MAX_LIN, VEL_MAX_LIN);

    float Kp_line = 0.05f;  // Correção proporcional
    float Kd_line = 0.3f;  // Resistência a oscilação (amortecedor)

    float w_cmd = (Kp_line * angle_error) - (Kd_line * current_w);

    if (fabs(angle_error) > 0.05f && fabs(w_cmd) < VEL_MIN_ANG)
        w_cmd = copysign(VEL_MIN_ANG, w_cmd);

    w_cmd = constrain(w_cmd, -VEL_MAX_ANG, VEL_MAX_ANG);

    float L = WHEEL_BASE_M;
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
}

void DiffCar::navegar_reto(double dist){
    this->target_x = odom_real.x + dist;
    this->target_theta = odom_real.theta;
}

void DiffCar::debug_ekf() {
    Serial.print("EKF State: ");
    Serial.print(ekf.x[0], 4); Serial.print(", ");
    Serial.print(ekf.x[1], 4); Serial.print(", ");
    Serial.print(ekf.x[2], 4); Serial.print(", ");
    Serial.print(ekf.x[3], 4); Serial.print(", ");
    Serial.println(ekf.x[4], 4);
    Serial.print("IMU Yaw (rad): ");
    Serial.println(ypr_d[0], 4);
    Serial.print("Linear Vel (m/s): ");
    Serial.println(diffCar.odom_enc.linear_velocity, 4);
    Serial.print("Angular Vel (rad/s): ");
    Serial.println(diffCar.odom_enc.angular_velocity, 4);
    Serial.println();
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