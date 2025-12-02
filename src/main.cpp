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
    const TickType_t dt = pdMS_TO_TICKS(10); // 100 Hz
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
    static unsigned long last100 = 0;
    static unsigned long last10 = 0;
    static unsigned long last1s = 0;
    unsigned long now = micros();
    //diffCar.debug_raw_pins();
    // ============================
    //        100 Hz
    // ============================
    if (now - last100 >= 2000) { // 200 Hz = 2 ms
        last100 = now;
        // 1) velocity_update
        diffCar.velocity_update();
        // 3) motor handler
        diffCar.handler_motor();

    }

    // ============================
    //        10 Hz
    // ============================
    if (now - last10 >= 100000) { // 10 Hz = 100 ms
		const float dt = (now - last10) / 1000000.0f;
		mqtt.handler();
		diffCar.encoder_odometry_update();

        diffCar.ekf_step_with_gyro_rate(dt, diffCar.odom_enc.angular_velocity,diffCar.odom_enc.linear_velocity,diffCar.ypr_d[0]);

		//diffCar.debug_ekf();
        diffCar.debug_encoder();
		
		if(diffCar.mode == 1){
			diffCar.navigate_to(diffCar.target_x, diffCar.target_y, diffCar.target_theta, 0.05f, 0.4f);
        
			//diffCar.debug_nav();
		}
		last10 = now;
    }
    // ============================
    //        1 Hz
    // ============================
    if(now - last1s >= 1000000) { // 1 Hz = 1 s
        last1s = now;
        diffCar.led_rotate();
    }
}


