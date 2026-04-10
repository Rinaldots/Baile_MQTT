// Configurações do filtro de Kalman estendido
#define _float_t double


#define ENCODER_A_1 27
#define ENCODER_B_1 14

#define VEL_MIN_LIN 15.0f  // cm/s
#define VEL_MAX_LIN 60.0f   // cm/s

#define VEL_MIN_ANG 1.5f   // rad/s (vel min de 15 cm/s nas rodas p/ girar)
#define VEL_MAX_ANG 4.0f   // rad/s (vel máx de 30 cm/s nas rodas)

// Frequências de Atualização e Timers
#define MPU_TASK_MS     5       // 200 Hz (Tick do RTOS para a IMU)
#define LOOP_FAST_US    10000UL  // 100 Hz 
#define LOOP_NAV_US     20000UL //  50 Hz 
#define LOOP_MQTT_US    20000UL     // Se não receber pulso do encoder em 500ms, considera parado
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


#define PULSES_PER_REV 21.0f
#define WHEEL_CIRCUMFERENCE_CM 0.065f * 3.14159f * 100.0f
#define CENTIMETER_PER_PULSE (WHEEL_CIRCUMFERENCE_CM / PULSES_PER_REV)

#define NOBS 5

#define IMU_CONST  8192.0f*9.81f
// Leds 

#define led_latchPin  19
#define led_clockPin 18
#define led_dataPin 23

#define WHEEL_BASE_CM 20.0f
// Define qual tipo de encoder está sendo usado
//#define ENCODER_QUAD
#define ENCODER_SIMPLE