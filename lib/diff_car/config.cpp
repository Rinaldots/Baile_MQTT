// Configurações do filtro de Kalman estendido
#define EKF_N 5
#define EKF_M 2
#define _float_t double


#define ENCODER_A_1 27
#define ENCODER_B_1 14

#define VEL_MIN_LIN 0.3f  // m/s
#define VEL_MAX_LIN 0.4f   // m/s

#define VEL_MIN_ANG 0.3f   // rad/s
#define VEL_MAX_ANG 0.4f   // rad/s

#define C1 -2.47122952e-05
#define C2 1.22241432e-02
#define C3 -1.04095195e+00

#define MIN_PULSES_PER_S 0.5f 
#define NO_PULSE_TIMEOUT_MS 1000

// Motor A
#define MOTOR_IN1   25
#define MOTOR_IN2   26
// Motor B
#define MOTOR_IN3   32
#define MOTOR_IN4   33


#define PULSES_PER_REV 60.0f
#define WHEEL_CIRCUMFERENCE_M 0.065f * 3.14159f
#define METER_PER_PULSE (WHEEL_CIRCUMFERENCE_M / PULSES_PER_REV)

#define NOBS 5

// Leds 

#define led_latchPin  19
#define led_clockPin 18
#define led_dataPin 23

#define WHEEL_BASE_M 0.20f
// Define qual tipo de encoder está sendo usado
//#define ENCODER_QUAD
#define ENCODER_SIMPLE