#ifndef PID_ADAPTIVE_H
#define PID_ADAPTIVE_H

#include <cmath>
#include <Arduino.h>
// =============================
//   ESTIMADOR ADAPTATIVO MRAC
// =============================
struct MotorAdaptiveModel {
    float a_hat;
    float b_hat;
    float gamma_a;
    float gamma_b;
    float prev_speed;
    float prev_input;

    MotorAdaptiveModel()
        : a_hat(0.7), b_hat(0.2),
          gamma_a(0.0001), gamma_b(0.0001),
          prev_speed(0), prev_input(0)
    {}

    void update(float speed, float input)
    {
        float y_hat = a_hat * prev_speed + b_hat * prev_input;
        float error = speed - y_hat;

        a_hat += gamma_a * error * prev_speed;
        b_hat += gamma_b * error * prev_input;

        // limites do modelo (IMPORTANTE)
        a_hat = constrain(a_hat, 0.1f, 0.99f);
        b_hat = constrain(b_hat, 0.05f, 1.5f);

        prev_speed = speed;
        prev_input = input;
    }
};


// =============================
//      PID ADAPTATIVO COMPLETO
// =============================
class AdaptivePID {
public:
    AdaptivePID(float dt,
                float min_output, float max_output,
                float *target, float *measured_value, float *output)
        : dt(dt), min_output(min_output), max_output(max_output),
          target(target), measured_value(measured_value), output(output),
          integral(0), prev_error(0)
    {}

    bool compute()
    {
        // 1 — Atualizar modelo adaptativo
        model.update(*measured_value, last_output);

        // 2 — Ajustar ganhos automaticamente
        float b = fabs(model.b_hat);

        Kp = 80.0f * b;   // 
        Ki =  8.0f * b;   // 
        Kd =  0.2f / b;  // 

        // ---------------------------
        // 3 — PID normal + anti-windup
        // ---------------------------
        float error = (*target - *measured_value) * 100.0f;
        
        float proportional = Kp * error;
        float derivative = Kd * (error - prev_error) / dt;

        float new_integral = integral + error * dt;
        float raw_output = proportional + Ki * new_integral + derivative;

        // saturação
        float clamped = constrain(raw_output, min_output, max_output);
        if (fabs(raw_output - clamped) < 1e-5 || (error * raw_output) < 0) {
            integral = new_integral;
        }

        // -----------------------------------------
        // 5 — Compensação de deadzone
        // -----------------------------------------
        if (fabs(clamped) < 25 && fabs(error) > 1.0f) {
            clamped = 25 * copysign(1.0f, clamped);
        }

        prev_error = error;
        last_output = clamped;
        *output = clamped;

        return true;
    }

    void reset() {
        integral = 0;
        prev_error = 0;
    }

    // valores adaptados (opcional)
    float Kp, Ki, Kd;

private:
    MotorAdaptiveModel model;

    float *target;
    float *measured_value;
    float *output;

    float dt;
    float min_output, max_output;

    float integral;
    float prev_error;
    float last_output = 0;
};

#endif // PID_ADAPTIVE_H
