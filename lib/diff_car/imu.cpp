#include "diff_car.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

bool DMPReady = false;  // Set true if DMP init was successful
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
VectorInt16 accel;      // [x, y, z]            Accelerometer measurements
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and
VectorInt16 gyro;       // [x, y, z]            Gyroscope measurements
VectorInt16 aa;        // Accel raw
VectorInt16 aaWorld;   // Accel in World frame (gravity removed and rotated)

void DiffCar::setup_mpu() {
    Wire.begin();

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing MPU6050 connection..."));
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (true);
    }
    Serial.println("MPU6050 connection successful");

    Serial.println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println("Offsets:");
        mpu.PrintActiveOffsets();

        mpu.setDMPEnabled(true);

        Serial.println("DMP ready (no interrupt mode)");
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

void DiffCar::mpu_update() {
    if (!DMPReady) return;

    // Use dmpGetCurrentFIFOPacket() to get the latest packet directly.
    // It's overflow proof and guarantees aligned bytes (prevents huge garbage spikes)
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        mpu.dmpGetQuaternion(&q, FIFOBuffer);

        // --- QUATERNION ---
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        mpu.dmpGetGravity(&gravity, &q);
        
        mpu.dmpGetAccel(&aa, FIFOBuffer);
        
        mpu.dmpGetLinearAccel(&accel, &aa, &gravity);
        
        // Transforma a aceleração linear (sem gravidade) para a referência do mundo (norte/sul/vertical absoluto)
        // Implementação manual da rotação do Quaternion
        aaWorld.x = (accel.x * (1 - 2*q.y*q.y - 2*q.z*q.z)) + (accel.y * (2*q.x*q.y - 2*q.w*q.z)) + (accel.z * (2*q.x*q.z + 2*q.w*q.y));
        aaWorld.y = (accel.x * (2*q.x*q.y + 2*q.w*q.z)) + (accel.y * (1 - 2*q.x*q.x - 2*q.z*q.z)) + (accel.z * (2*q.y*q.z - 2*q.w*q.x));
        aaWorld.z = (accel.x * (2*q.x*q.z - 2*q.w*q.y)) + (accel.y * (2*q.y*q.z + 2*q.w*q.x)) + (accel.z * (1 - 2*q.x*q.x - 2*q.y*q.y));
        
        mpu.dmpGetGyro(&gyro, FIFOBuffer);

        // Usa aaWorld purificado do vetor de gravidade e alinhado aos eixos do chão:
        diffCar.accel_d[0] = aaWorld.x / IMU_CONST;
        diffCar.accel_d[1] = aaWorld.y / IMU_CONST;
        diffCar.accel_d[2] = aaWorld.z / IMU_CONST;
        diffCar.gyro_d[0] = gyro.x;
        diffCar.gyro_d[1] = gyro.y;
        diffCar.gyro_d[2] = gyro.z;
        diffCar.ypr_d[0] = -ypr[0]; // Sinal invertido para casar com o sistema do robô
    }
}


void DiffCar::mpu_covariance(){
    const int N = 3000;
    Serial.println("Calculando covariância do acelerômetro...");
    float accel_mean[3] = {0, 0, 0};
    float accel_M2[3] = {0, 0, 0};

    for (int i = 0; i < N; i++) {
        // mpu_update() is already being called continuously by mpu_task on Core 0!
        // mpu_update();

        for (int j = 0; j < 3; j++) {
            float x = accel_d[j];
            float delta = x - accel_mean[j];
            accel_mean[j] += delta / (i + 1);
            accel_M2[j] += delta * (x - accel_mean[j]);
        }

        delay(10);
    }

    Serial.println("Covariância do Acelerômetro:");
    for (int j = 0; j < 3; j++) {
        float variance = accel_M2[j] / (N - 1);
        Serial.printf("Eixo %d: %.6f\n", j, variance);
    }
}
