#ifndef FUZZY_CONTROL_H
#define FUZZY_CONTROL_H

#include <cmath>
#include <Arduino.h>

class FuzzyController {
public:
    FuzzyController(float out_min, float out_max, float *tgt, float *meas, float *out)
        : min_output(out_min), max_output(out_max),
          target(tgt), measured(meas), output(out) {}

    void compute() {
        // Cálculo do tempo para normalizar a derivada (Indispensável para robustez)
        unsigned long now = micros();
        float dt = (now - last_time) / 1000000.0f;
        if (dt <= 0 || dt > 0.5f) dt = 0.01f; // Failsafe para o primeiro ciclo
        last_time = now;

        float e = *target - *measured;
        float de = (e - prev_error) / dt; // Derivada real baseada no tempo
        prev_error = e;

        // 1. Fuzzificação com Ganho de Escala (Normalização)
        // Erro: N (Negativo), Z (Zero), P (Positivo)
        float e_N = trapmf(e, -2.0, -1.0, -0.4, -0.05); // Trapezoidal para erro grande
        float e_Z = trimf(e, -0.1, 0.0, 0.1);
        float e_P = trapmf(e, 0.05, 0.4, 1.0, 2.0);

        // Delta Erro: N, Z, P
        float de_N = trapmf(de, -5.0, -2.0, -1.0, -0.1);
        float de_Z = trimf(de, -0.2, 0.0, 0.2);
        float de_P = trapmf(de, 0.1, 1.0, 2.0, 5.0);

        // 2. Base de Regras & Defuzzificação
        float out_weight = 0.0;
        float weight_sum = 0.0;

        auto rule = [&](float w1, float w2, float out_val) {
            float activation = fmin(w1, w2);
            if (activation > 0) {
                out_weight += activation * out_val;
                weight_sum += activation;
            }
        };

        // Regras de Matriz Fuzzy otimizadas
        rule(e_N, de_N, -1.0); rule(e_N, de_Z, -0.6); rule(e_N, de_P, -0.2);
        rule(e_Z, de_N, -0.3); rule(e_Z, de_Z,  0.0); rule(e_Z, de_P,  0.3);
        rule(e_P, de_N,  0.2); rule(e_P, de_Z,  0.6); rule(e_P, de_P,  1.0);

        float control_action = 0.0;
        if (weight_sum > 0.001f) {
            control_action = out_weight / weight_sum;
        }

        // 3. Auto-Tuning com Filtro de Passa-Baixa no ganho
        updateAutoTuning(e);

        // Integração do sinal com Anti-Windup
        // last_output aqui atua como o termo integral do PID
        last_output += control_action * max_increment;
        
        // Anti-Windup: Se o motor já está no talo, para de integrar
        if (last_output > max_output) last_output = max_output;
        else if (last_output < min_output) last_output = min_output;

        *output = last_output;
    }

    void reset() {
        prev_error = 0;
        last_output = 0;
        same_sign_count = 0;
    }

private:
    float *target, *measured, *output;
    float min_output, max_output;
    float prev_error = 0, last_output = 0;
    unsigned long last_time = 0;

    // Tuning Params
    float max_increment = 0.005f;
    int same_sign_count = 0;
    float prev_e_sign = 0;

    void updateAutoTuning(float e) {
        float e_sign = (e > 0.01f) ? 1.0f : ((e < -0.01f) ? -1.0f : 0.0f);

        if (e_sign != 0 && prev_e_sign != 0 && e_sign != prev_e_sign) {
            // Cruzou o zero: Reduz o ganho para evitar overshoot
            max_increment *= 0.85f; 
            same_sign_count = 0;
        } else if (fabs(e) > 0.05f) {
            same_sign_count++;
            if (same_sign_count > 30) {
                // Muito tempo longe do alvo: Aumenta o ganho (mais torque)
                max_increment *= 1.05f;
                same_sign_count = 0;
            }
        }
        max_increment = constrain(max_increment, 0.001f, 0.04f);
        prev_e_sign = e_sign;
    }

    // Triangular membership
    float trimf(float x, float a, float b, float c) {
        return fmax(0.0f, fmin((x - a) / (b - a), (c - x) / (c - b)));
    }

    // Trapezoidal membership (Muito mais robusto para extremos)
    float trapmf(float x, float a, float b, float c, float d) {
        return fmax(0.0f, fmin(fmin((x - a) / (b - a), 1.0f), (d - x) / (d - c)));
    }
};

#endif