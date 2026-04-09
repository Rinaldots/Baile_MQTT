
#include "diff_car.h"

#include <stdio.h>
#include <math.h>

DiffCar::DiffCar() {}

static double wrapAngleD(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

void DiffCar::setup() {
  setup_encoder();
  setup_h_bridge();
  setup_mpu();
  setup_leds();
  update_leds(0,1,0,1,0,1,0,1);
}

void DiffCar::encoder_odometry_update() {
    static unsigned long last_odom_time = micros();
    unsigned long current_time = micros();
    float dt = (current_time - last_odom_time) / 1000000.0f;
    last_odom_time = current_time;

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

    // ==============================================================
    // Filtro de Kalman 1D para fundir odom_enc e ypr_d[0] (IMU)
    // ==============================================================
    static float theta_k = 0.0f; // Estado estimado (Ângulo)
    static float P_k = 1.0f;     // Covariância do Erro
    
    // Parâmetros de Tuning do Kalman (Variâncias Mínimas)
    float Q = 0.001f; // Ruído de Processo (Incerteza do Encoder)
    float R = 0.05f;  // Ruído de Medição (Incerteza do Yaw da IMU)

    // 1. Predição (Prediction Step) -> O que os encoders acham que aconteceu?
    float theta_pred = theta_k + (omega * dt);
    float P_pred = P_k + Q;

    // 2. Coleta a medição da IMU garantindo não pular de 180 pra -180 bruscamente
    float z_imu = ypr_d[0];
    
    // Calcula a inovação do erro pelo caminho mais curto no círculo
    float y_inovacao = wrapAngleD(z_imu - theta_pred);

    // 3. Atualização (Update Step) -> Ganho de Kalman
    float K = P_pred / (P_pred + R);
    
    // 4. Estado final: Onde realmente estamos
    theta_k = wrapAngleD(theta_pred + (K * y_inovacao));
    P_k = (1.0f - K) * P_pred;
    
    // Atrita Odometria Real (Yaw)
    odom_real.theta = theta_k;

    // ==============================================================
    // Filtro de Kalman 1D para fundir Velocidade Linear (IMU + Encoders)
    // ==============================================================
    // A IMU não consegue prever Posição (X) direto sem virar um caos de drift,
  
    static float v_k = 0.0f;     
    static float Pv_k = 1.0f;    
    
    float Q_v = 0.001f; // Incerteza da aceleração da IMU (Ruído de Processo)
    float R_v = 0.5f;  // Incerteza do Encoder (Ruído de Medição)
    
    float imu_a_forward = accel_d[1]; 

    // 1. Predição: A IMU diz o quanto a velocidade mudou baseado na força G
    float v_pred = v_k + (imu_a_forward * dt);
    float Pv_pred = Pv_k + Q_v;

    // 2. Medição: O Encoder diz o quanto a roda girou
    float z_v = v; // v = (vl+vr)/2.0f
    float y_v_inovacao = z_v - v_pred;

    // 3. Atualização (Update Step)
    float K_v = Pv_pred / (Pv_pred + R_v);
    v_k = v_pred + (K_v * y_v_inovacao);
    Pv_k = (1.0f - K_v) * Pv_pred;

    // Atualiza a posição X e Y integrando a velocidade fundida e o rumo fundido
    if (dt > 0.0f && dt < 0.2f) { // Failsafe para pausadas longas
        odom_real.x += v_k * cosf(odom_real.theta) * dt;
        odom_real.y += v_k * sinf(odom_real.theta) * dt;
    }
    odom_real.linear_velocity = v_k;
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
    float dy = target_y - odom_real.y;
    float distance = sqrt(dx * dx + dy * dy);

    float current_w = odom_enc.angular_velocity; 

    // Fase 1: Chegar no X,Y (distância < precisão)
    // Se chegou na coordenada, Fase 2: Girar no próprio eixo pro ângulo final (target_theta)
    if (distance < precision)
    {
        float final_angle_error = wrapAngleD(target_theta - odom_real.theta);
        
        float Kp_rot = 2.0f; 
        float Kd_rot = 0.2f; 
        float w = (Kp_rot * final_angle_error) - (Kd_rot * current_w);
        
        // Garante torque mínimo se travar
        if (fabs(final_angle_error) > 0.04f && fabs(w) < VEL_MIN_ANG)
            w = copysign(VEL_MIN_ANG, w);
            
        w = constrain(w, -VEL_MAX_ANG, VEL_MAX_ANG);
        
        float L = WHEEL_BASE_M;
        left_velocity_target  = -w * L * 0.5f;
        right_velocity_target =  w * L * 0.5f;

        // Se alinhou perfeitamente, desliga e finaliza modo de navegação
        if (fabs(final_angle_error) < 0.1f) {
            left_velocity_target = 0.0f;
            right_velocity_target = 0.0f;
            diffCar.mode = 0; 
        }
        return;
    }

    // Calcula o rumo até a posição (vetor XY rumo ao Target)
    float path_angle = atan2(dy, dx);
    float angle_error = wrapAngleD(path_angle - odom_real.theta);
    
    // Controle polar
    // Acelera gradativo, mas zera a velocidade linear se ele estiver muito virado pros lados (> 45 graus / 0.7 rad) 
    // pro robô rodar no eixo primeiro antes de dar o bote.
    float v_cmd = target_velocity * tanh(distance * 2.0f); // Modificado para chegar + rapido
    if (fabs(angle_error) > 0.6f) {
        v_cmd *= 0.1f; // Corta 90% da lin. vel pra forçar girar primeiro 
    }
    
    // Garante que se tiver distante o suficiente, envia pelo menos a min lin
    if(distance > precision && fabs(v_cmd) < VEL_MIN_LIN && fabs(angle_error) < 0.6f) {
        v_cmd = copysign(VEL_MIN_LIN, v_cmd);
    }
    
    v_cmd = constrain(v_cmd, -VEL_MAX_LIN, VEL_MAX_LIN);

    float Kp_line = 3.0f;  
    float Kd_line = 0.3f;  

    float w_cmd = (Kp_line * angle_error) - (Kd_line * current_w);

    if (fabs(angle_error) > 0.05f && fabs(w_cmd) < VEL_MIN_ANG)
        w_cmd = copysign(VEL_MIN_ANG, w_cmd);

    w_cmd = constrain(w_cmd, -VEL_MAX_ANG, VEL_MAX_ANG);

    float L = WHEEL_BASE_M;
    float vl = v_cmd - (w_cmd * L * 0.5f);
    float vr = v_cmd + (w_cmd * L * 0.5f);

    // Normaliza velocidade se bater no limite do motor
    float max_vel = fmax(fabs(vl), fabs(vr));
    if (max_vel > VEL_MAX_LIN) {
        float scale = VEL_MAX_LIN / max_vel;
        vl *= scale;
        vr *= scale;
    }

    left_velocity_target  = vl;
    right_velocity_target = vr;
}


void DiffCar::mover_distancia(float metros, float velocidade_linear) {
    float x_inicial = odom_real.x;
    float y_inicial = odom_real.y;
    float ang_inicial = odom_real.theta; // Usa a IMU como referência de rumo (Yaw)
    
    // Converte a intenção linear baseada no parâmetro positivo/negativo "metros"
    float dist_alvo = fabs(metros);
    float vel = (metros < 0.0f) ? -fabs(velocidade_linear) : fabs(velocidade_linear);
    
    float dist_percorrida = 0.0f;
    
    // Control loop bloqueante assíncrono à thread principal da aplicação
    while (dist_percorrida < dist_alvo) {
        
        // Mantém a sensoriamento girando em caso do "delay" impedir a main() global
        velocity_update();
        mpu_update();
        encoder_odometry_update();

        // Pitágoras para calcular o quanto se moveu no mundo real (em metros)
        float dx = odom_real.x - x_inicial;
        float dy = odom_real.y - y_inicial;
        dist_percorrida = sqrt(dx*dx + dy*dy);
        
        // Rampa de Desaceleração suave nos últimos 15 cm 
        // para dar espaço à frenagem e evitar "skipping" sobre o ponto alvo
        float vel_calc = vel;
        float erro_dist = dist_alvo - dist_percorrida;
        if (erro_dist < 0.15f) {
            vel_calc = vel * (erro_dist / 0.15f);
            
            // Garante que não zera de vez e não tenha força para rolar a bucha do pneu, travando no caminho
            if (fabs(vel_calc) < 0.15f) {
                vel_calc = copysign(0.15f, vel); 
            }
        }
        
        manter_rumo(vel_calc, ang_inicial);
        
        // Comando do driver para acionar PWMs novos localmente na Task
        handler_motor();
        delay(10); // Pausa de 10ms (100Hz) igual aos ciclos de telemetria base
    }
    
    // Chegou no alvo: Corta os motores imediatamente para estancar atrito
    left_velocity_target = 0.0f;
    right_velocity_target = 0.0f;
    handler_motor(); // Força update da ponte H uma última vez
}

void DiffCar::manter_rumo(float velocidade_linear, float angulo_desejado) {
    // 1. Cálculo do Erro
    float erro_theta = wrapAngleD(angulo_desejado - ypr_d[0]);

    // 2. Parâmetros do PID (Ajuste esses valores!)
    static float erro_acumulado = 0;
    float Kp = 5.0f;   // Resposta imediata
    float Ki = 0.05f;  // Corrige desvios persistentes
    
    // 3. Cálculo do Termo Integral com Anti-Windup
    // Só acumulamos o erro se o robô não estiver saturado (PWM no máximo)
    erro_acumulado += erro_theta;
    
    // Limita o erro acumulado (Clamping) para evitar o "Windup"
    erro_acumulado = constrain(erro_acumulado, -0.5f, 0.5f); 

    // 4. Cálculo da Velocidade Angular (w)
    float w_correcao = (erro_theta * Kp) + (erro_acumulado * Ki);
    
    // Limita a correção para não girar rápido demais
    w_correcao = constrain(w_correcao, -VEL_MAX_ANG, VEL_MAX_ANG);

    // 5. Mixagem nos motores (Cinemática Diferencial)
    float L = WHEEL_BASE_M;
    left_velocity_target  = velocidade_linear - (w_correcao * L * 0.5f);
    right_velocity_target = velocidade_linear + (w_correcao * L * 0.5f);

    // Se o alvo for parar, zeramos o acumulador para a próxima manobra
    if (velocidade_linear == 0 && fabs(erro_theta) < 0.05f) {
        erro_acumulado = 0;
    }
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