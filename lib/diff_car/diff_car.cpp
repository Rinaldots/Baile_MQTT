
#include "diff_car.h"

#include <stdio.h>
#include <math.h>


DiffCar::DiffCar() {}

void DiffCar::setup() {
  //setup_mpu();
  //calibrate_imu();
  setup_h_bridge();
  setup_encoder();
}

// Função para enviar dados de telemetria via Bluetooth
