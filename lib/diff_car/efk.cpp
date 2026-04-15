#include "diff_car.h"
#include <math.h>
#include <stdio.h>

static double wrapAngleD(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

void DiffCar::ekf_update(float dc, float dtheta, float v, float dt) {

    static float X[4] = {0,0,0,0}; // x, y, theta, v

    static float P[4][4] = {
        {1,0,0,0},
        {0,1,0,0},
        {0,0,0.5f,0},
        {0,0,0,1}
    };

    const float Q[4] = {10.0f, 10.0f, 0.01f, 20.0f};
    const float R[2] = {0.05f, 5.0f};

    // ===== PREDIÇÃO =====
    float theta_mid = X[2] + 0.5f * dtheta;

    float Xp[4];
    Xp[0] = X[0] + dc * cosf(theta_mid);
    Xp[1] = X[1] + dc * sinf(theta_mid);
    Xp[2] = wrapAngleD(X[2] + dtheta);
    Xp[3] = v;

    float F[4][4] = {
        {1, 0, -dc * sinf(theta_mid), cosf(theta_mid) * dt},
        {0, 1,  dc * cosf(theta_mid), sinf(theta_mid) * dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    float FP[4][4] = {0};
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            for(int k=0;k<4;k++)
                FP[i][j] += F[i][k]*P[k][j];

    float Pp[4][4] = {0};
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++){
            for(int k=0;k<4;k++)
                Pp[i][j] += FP[i][k]*F[j][k];
            if(i==j) Pp[i][j] += Q[i];
        }

    // ===== MEDIÇÃO =====
    float z_theta = ypr_d[0];
    float z_v = v;

    float y[2];
    y[0] = wrapAngleD(z_theta - Xp[2]);
    y[1] = z_v - Xp[3];

    float S[2][2];
    S[0][0] = Pp[2][2] + R[0];
    S[0][1] = Pp[2][3];
    S[1][0] = Pp[3][2];
    S[1][1] = Pp[3][3] + R[1];

    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (fabs(det) < 1e-6f) return;

    float S_inv[2][2];
    S_inv[0][0] =  S[1][1]/det;
    S_inv[0][1] = -S[0][1]/det;
    S_inv[1][0] = -S[1][0]/det;
    S_inv[1][1] =  S[0][0]/det;

    float K[4][2];
    for(int i=0;i<4;i++){
        float p_theta = Pp[i][2];
        float p_v     = Pp[i][3];

        K[i][0] = p_theta*S_inv[0][0] + p_v*S_inv[1][0];
        K[i][1] = p_theta*S_inv[0][1] + p_v*S_inv[1][1];
    }

    for(int i=0;i<4;i++)
        X[i] = Xp[i] + K[i][0]*y[0] + K[i][1]*y[1];

    X[2] = wrapAngleD(X[2]);

    float I_KH[4][4] = {0};
    for(int i=0;i<4;i++){
        I_KH[i][i] = 1.0f;
        I_KH[i][2] -= K[i][0];
        I_KH[i][3] -= K[i][1];
    }

    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++){
            P[i][j] = 0;
            for(int k=0;k<4;k++)
                P[i][j] += I_KH[i][k]*Pp[k][j];
        }

    // ===== OUTPUT =====
    odom_real.x = X[0];
    odom_real.y = X[1];
    odom_real.theta = X[2];
    odom_real.linear_velocity = X[3];
}