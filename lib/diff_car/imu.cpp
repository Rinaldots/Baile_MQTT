#include "diff_car.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
VectorInt16 accel;      // [x, y, z]            Accelerometer measurements
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and
VectorInt16 gyro;       // [x, y, z]            Gyroscope measurements
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
VectorInt16 aa;        // Accel raw
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

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

    uint16_t fifoCount = mpu.getFIFOCount();

    // Se FIFO estourar → reset
    if (fifoCount == 1024) {
        mpu.resetFIFO();
        return;
    }

    // Só processa se tiver pacote completo
    if (fifoCount < packetSize) return;

    // Ler um pacote do FIFO
    mpu.getFIFOBytes(FIFOBuffer, packetSize);

    mpu.dmpGetQuaternion(&q, FIFOBuffer);

    // --- QUATERNION ---
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    mpu.dmpGetGravity(&gravity, &q);
    
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    
    mpu.dmpGetLinearAccel(&accel, &aa, &gravity);
    
    mpu.dmpGetGyro(&gyro, FIFOBuffer);
    diffCar.accel_d[0] = accel.x/8192;
    diffCar.accel_d[1] = accel.y/8192;
    diffCar.accel_d[2] = accel.z/8192;
    diffCar.gyro_d[0] = gyro.x;
    diffCar.gyro_d[1] = gyro.y;
    diffCar.gyro_d[2] = gyro.z;
    diffCar.ypr_d[0] = -ypr[0];
    
}



