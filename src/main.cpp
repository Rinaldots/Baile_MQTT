#include <Arduino.h>
#include <diff_car.h>
#include <mqtt.h>
extern "C" {
#include "esp_core_dump.h"
}
DiffCar diffCar;
MqttTask mqtt;
static void clearStaleCoreDump() {
#ifdef CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
	esp_core_dump_image_erase();
#endif
}

void mpu_task(void *pvParameters)
{
    const TickType_t dt = pdMS_TO_TICKS(MPU_TASK_MS); // Tempo configurado em config.cpp
    while (true) {
        diffCar.mpu_update();
        vTaskDelay(dt);
    }
}

void setup() {
	Serial.begin(115200);
	Serial.println("Starting...");
	clearStaleCoreDump();
	
	mqtt.setup();

	diffCar.setup();

	xTaskCreatePinnedToCore(
        mpu_task,               // função
        "MPU Task",             // nome
        4096,                   // stack
        NULL,                   // params
        1,                      // prioridade
        NULL,                   // task handle
        0                       // CORE 0 <--------
    );

	diffCar.update_leds(1,1,0,0,0,0,0,0);
	Serial.println("Setup complete. BLE task running on Core 0");
}



void loop() {
    static unsigned long lastFast = 0;
    static unsigned long lastNav = 0;
    static unsigned long last1s = 0;
    unsigned long now = micros();
    
    // ============================
    //        Malha Rápida (Cinemática e Motores)
    // ============================
    if (now - lastFast >= LOOP_FAST_US) { 
        lastFast = now;
        diffCar.velocity_update();
        diffCar.encoder_odometry_update();
        diffCar.handler_motor();
    }

    // ============================
    //        Malha de Navegação (Navegação PID e MQTT)
    // ============================
    if (now - lastNav >= LOOP_NAV_US) {
		lastNav = now;
		
		mqtt.handler();
		
        //diffCar.debug_encoder();

		mqtt.updateTelemetry();

		if(diffCar.mode == 1){
			// Alvo XY e angulo | Tolerância coordenada | Velocidade Linear escalar
			diffCar.navigate_to(diffCar.target_x, diffCar.target_y, diffCar.target_theta, 0.1f, 0.40f);
		}
    }
    // ============================
    //        1 Hz
    // ============================
    if(now - last1s >= 1000000) { // 1 Hz = 1 s
        last1s = now;
        
        diffCar.led_rotate();
    }
}


