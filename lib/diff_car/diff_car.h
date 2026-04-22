#ifndef DIFF_CAR_H
#define DIFF_CAR_H

#include "config.cpp"
#include <Arduino.h>
#include <Preferences.h>
#include "MPU6050_6Axis_MotionApps20.h"
// geometry_msgs from ROS2

// geometry_msgs from ROS2
struct MotorPwmResult {
    float left;
    float right;
};
struct Odometry {
    float x;
    float y;
    float theta;
    float absolute_velocity;
    float linear_velocity;
    float angular_velocity;
};

class DiffCar {
public:
    DiffCar();
    
    //
    // ============================
    // IMU
    // ============================
    MPU6050 mpu;
    bool calibrated = false;
    bool has_imu = false;
    float ypr_d[3]   = {0,0,0};
    float accel_d[3] = {0,0,0};
    float gyro_d[3]  = {0,0,0};

    // ============================
    // Motores
    // ============================
    float left_motor_pwm  = 0.0f;
    float right_motor_pwm = 0.0f;
    int left_motor_dir  = 1;
    int right_motor_dir = 1;
    bool tuning_mode = false; // flag para auto-tune
    float min_pwm_left = 771.0f;
    float min_pwm_right = 771.0f;

    // ============================
    // Encoder → PCNT MODE
    // ============================
    // Saídas filtradas
    float left_freq_filtered  = 0.0f;
    float right_freq_filtered = 0.0f;

    float left_freq_raw  = 0.0f;
    float right_freq_raw = 0.0f;

    // Snapshot para calcular dCount
    int16_t left_count_snapshot  = 0;
    int16_t right_count_snapshot = 0;

    volatile uint32_t right_total_pulses = 0;
    volatile uint32_t left_total_pulses = 0;
    // Tempo do último cálculo de velocidade
    unsigned long last_vel_update_us = 0;

    static constexpr float MIN_PULSES = 0.5f;

    // ============================
    // RESULTADO — Velocidade estimada
    // ============================
    float left_velocity_cms  = 0.0f;
    float right_velocity_cms = 0.0f;

    float abs_left_velocity_cms  = 0.0f;
    float abs_right_velocity_cms = 0.0f;
    // Velocidade alvo (m/s)
    float left_velocity_target  = 0.0f;
    float right_velocity_target = 0.0f;


    float abs_left_velocity_target  = 0.0f;
    float abs_right_velocity_target = 0.0f;
    
    float left_pid_output  = 0.0f;
    float right_pid_output = 0.0f;

    // ============================
    // Navegação
    // ============================
    int mode = 0;
    double target_x = 0.0;
    double target_y = 0.0;
    double target_theta = 0.0;

    // ============================
    void setup();

    // Setup functions
    void setup_encoder();
    void setup_h_bridge();
    void setup_mpu();

    // H-Bridge functions
    void update_h_bridge();
    void set_motor_speed(float left_motor_pwm, float right_motor_pwm);
    void handler_motor();
    void calibrate_motors_inertia();
    void auto_tune_pid();
    // Encoder functions
    void debug_encoder();
    void velocity_update();
    void debug_raw_pins();
    // IMU functions
    void mpu_update();
    void mpu_covariance();
    // Leds functions
    byte datArray[8] = {0,0,0,0,0,0,0,0};
    void setup_leds();
    void update_leds(bool led1, bool led2, bool led3, bool led4, bool led5, bool led6, bool led7, bool led8);
    void led_rotate();

    // Odometry functions
    Odometry odom_enc;
    Odometry odom_real;
    Odometry odom_referencial;
    
    // roda esquerda
    float left_kalman_x = 0.0f;
    float left_kalman_P = 1.0f;

    // roda direita
    float right_kalman_x = 0.0f;
    float right_kalman_P = 1.0f;

    void encoder_odometry_update();
    float kalman_update(float measurement, float &x, float &P);
    
    // Navigation functions
    //void navigate_to(float target_x, float target_y, float target_theta, float precision, float target_velocity);
    void navigate_to_target_pure_pursuit(float target_x, float target_y, float target_theta, float precision, float target_velocity);
    void debug_nav();
    void manter_rumo(float velocidade_linear, float angulo_desejado);
    void ekf_update(float dc, float dtheta, float v, float dt);
};

extern DiffCar diffCar;

#endif
