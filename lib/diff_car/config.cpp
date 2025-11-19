// Configurações do filtro de Kalman estendido
#define EKF_N 3
#define EKF_M 5
#define _float_t double

// Configurações do encoder
const float wheel_diameter_m = 0.065; // opcional para converter em m/s
const unsigned long DEBOUNCE_US = 1000; // rejeita pulsos < 1 ms (ajuste)
const unsigned long SAMPLE_MS = 100;    // janela principal de cálculo (100 ms)
const float EMA_ALPHA = 0.3f;           // 0..1, maior = resposta mais rápida

#define ENCODER_A_1 27
#define ENCODER_B_1 14


#define C1 -2.47122952e-05
#define C2 1.22241432e-02
#define C3 -1.04095195e+00

#define MIN_PULSES_PER_S 0.01f 
#define NO_PULSE_TIMEOUT_MS 1000

// Motor A
#define MOTOR_IN1   25
#define MOTOR_IN2   26
// Motor B
#define MOTOR_IN3   32
#define MOTOR_IN4   33


#define PULSES_PER_REV 16.0f
#define WHEEL_CIRCUMFERENCE_M 0.065f * 3.14159f

#define NOBS 5


// Define qual tipo de encoder está sendo usado
//#define ENCODER_QUAD
#define ENCODER_SIMPLE