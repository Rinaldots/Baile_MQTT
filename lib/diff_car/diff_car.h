#ifndef DIFF_CAR_H
#define DIFF_CAR_H

#include "config.cpp"
#include <Arduino.h>
#include <Preferences.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <QuickPID.h>

extern QuickPID left_pid;
extern QuickPID right_pid;
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

    // Tempo do último cálculo de velocidade
    unsigned long last_vel_update_us = 0;

    // thresholds / filtros
    static constexpr float HYST_UP   = 0.25f;
    static constexpr float HYST_DOWN = 0.15f;
    static constexpr unsigned long TIMEOUT_MS = 200;
    static constexpr float MIN_PULSES = 0.5f;

    // ============================
    // RESULTADO — Velocidade estimada
    // ============================
    float left_velocity_ms  = 0.0f;
    float right_velocity_ms = 0.0f;

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
    void setup_encoder_hall();
    void setup_encoder();
    void setup_h_bridge();
    void setup_mpu();

    // H-Bridge functions
    void update_h_bridge();
    void set_motor_speed(float left_motor_pwm, float right_motor_pwm);
    void set_motor_speed_ms(float vel_left, float vel_right);
    void handler_motor();
    MotorPwmResult set_motor_speed_msr(float vel_left, float vel_right);

    // Encoder functions
    void debug_encoder();
    void velocity_update();
    void debug_raw_pins();
    // IMU functions
    void mpu_update();

    uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

    // Leds functions
    byte datArray[8] = {0,0,0,0,0,0,0,0};
    void setup_leds();
    void update_leds(bool led1, bool led2, bool led3, bool led4, bool led5, bool led6, bool led7, bool led8);
    void led_rotate();

    // Odometry functions
    Odometry odom_enc;
    Odometry odom_real;
    void encoder_odometry_update();

    // EKF functions
    void ekf_step_with_gyro_rate(float dt, double gyro_z, float v_enc, double yaw_meas_rad);
    void ekf_init_diffcar();
    void debug_ekf();

    // Navigation functions
    void navigate_to(float target_x, float target_y, float target_theta, float precision, float target_velocity);
    void navigate_delta(float delta_x, float delta_y, float delta_theta);
    void debug_nav();
    void navegar_reto(double dist);
};

extern DiffCar diffCar;

#endif