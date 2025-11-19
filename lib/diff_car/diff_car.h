#ifndef DIFF_CAR_H
#define DIFF_CAR_H

#include "config.cpp"
#include "PID.cpp"
#include <Arduino.h>
#include <Preferences.h>

extern AdaptivePID left_pid;
extern AdaptivePID right_pid;
// geometry_msgs from ROS2

// geometry_msgs from ROS2
struct MotorPwmResult {
    float left;
    float right;
};

class DiffCar {
public:
    DiffCar();
    bool calibrated = false;

    float left_motor_pwm = -1;
    int left_motor_dir = -1;
    float right_motor_pwm = -1;
    int right_motor_dir = -1;

    volatile long encoder_left_count = 0;
    volatile long encoder_right_count = 0;
    volatile unsigned long left_last_pulse_us = 0;
    volatile unsigned long right_last_pulse_us = 0;
    volatile unsigned long left_last_interval_us = 0;
    volatile unsigned long right_last_interval_us = 0;
    int left_idx = 0;
    int right_idx = 0;
    float left_freq_filtered = 0.0;
    float right_freq_filtered = 0.0;
    unsigned long last_sample_time_ms = 0;
    long last_count_left_snapshot = 0;
    long last_count_right_snapshot = 0;

    // Velocidade em m/s
    float left_velocity_ms = 0.0;
    float right_velocity_ms = 0.0;
    bool right_stopped = false;
    bool left_stopped = false;
    unsigned long right_last_pulse_ms = 0;
    unsigned long left_last_pulse_ms = 0;

    // Velocidade alvo em m/s
    float left_velocity_target = 0.0;
    float right_velocity_target = 0.0;
    float left_pid_output = 0.0f;
    float right_pid_output = 0.0f;

    void setup();

    // Setup functions
    void setup_encoder_hall();
    void setup_encoder();
    void setup_h_bridge();

    // H-Bridge functions
    void update_h_bridge();
    void set_motor_speed(float left_motor_pwm, float right_motor_pwm);
    void set_motor_speed_ms(float vel_left, float vel_right);
    void handler_motor();

    // Encoder functions
    void debug_encoder();
    void velocity_update();
    MotorPwmResult set_motor_speed_msr(float vel_left, float vel_right);

};

extern DiffCar diffCar;

#endif