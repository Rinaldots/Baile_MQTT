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


void setup() {
	Serial.begin(115200);
	Serial.println("Starting...");
	clearStaleCoreDump();
	
	mqtt.setup();

	diffCar.setup();
	
	Serial.println("Setup complete. BLE task running on Core 0");
}



void loop() {
	static unsigned long last_t = 0;
	static unsigned long last_t1 = 0;

	unsigned long now = micros();

	if(now - last_t >= (unsigned long)(1000000 / 100)) { // 100 Hz
		last_t = now;
		// 1) velocity update
		diffCar.velocity_update();
		// 8) motor handler
		diffCar.handler_motor();
	}
	if(now - last_t1 >= (unsigned long)(1000000 / 1000)) { // 10 Hz
		last_t1 = now;
		//mqtt.updateTelemetry();
		mqtt.handler();
		diffCar.debug_encoder();
	}

}

