#include "diff_car.h"



void DiffCar::setup_leds() {
  pinMode(led_latchPin, OUTPUT);
  pinMode(led_clockPin, OUTPUT);
  pinMode(led_dataPin, OUTPUT);
}

void DiffCar::update_leds(bool led1, bool led2, bool led3, bool led4, bool led5, bool led6, bool led7, bool led8) {
    datArray[0] = led1 ? 1 : 0;
    datArray[1] = led2 ? 1 : 0;
    datArray[2] = led3 ? 1 : 0;
    datArray[3] = led4 ? 1 : 0;
    datArray[4] = led5 ? 1 : 0;
    datArray[5] = led6 ? 1 : 0;
    datArray[6] = led7 ? 1 : 0;
    datArray[7] = led8 ? 1 : 0;
    
    digitalWrite(led_latchPin, LOW);
    for (int i = 7; i >= 0; i--) {
        digitalWrite(led_clockPin, LOW);
        digitalWrite(led_dataPin, datArray[i]);
        digitalWrite(led_clockPin, HIGH);
    }
    digitalWrite(led_latchPin, HIGH);
}

void DiffCar::led_rotate() {
    // salva o primeiro bit
    bool first = datArray[0];

    // desloca todos para a esquerda
    for (int i = 0; i < 7; i++) {
        datArray[i] = datArray[i + 1];
    }

    // o primeiro vira o último
    datArray[7] = first;

    // envia para o 74HC595
    digitalWrite(led_latchPin, LOW);

    for (int i = 7; i >= 0; i--) {
        digitalWrite(led_clockPin, LOW);
        digitalWrite(led_dataPin, datArray[i] ? HIGH : LOW);
        digitalWrite(led_clockPin, HIGH);
    }

    digitalWrite(led_latchPin, HIGH);
}