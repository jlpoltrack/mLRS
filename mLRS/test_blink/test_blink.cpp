//*******************************************************
// Minimal RP2040 LED blink test
//*******************************************************

#include <Arduino.h>

void setup() {
    pinMode(25, OUTPUT);
}

void loop() {
    digitalWrite(25, HIGH);
    delay(200);
    digitalWrite(25, LOW);
    delay(200);
}
