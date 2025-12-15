#include "hardware/DMS15.h"
#include <Arduino.h>

DMS15::DMS15(int pin) : servoPin(pin) {}

bool DMS15::begin() {
    servo.attach(servoPin);
    Serial.printf("Servo attached to pin %d\n", servoPin);
    servo.write(110); 
    return servo.attached(); 
}

void DMS15::setAngle(int angle) {
    angle = constrain(angle, 0, 180); 
    servo.write(angle);
    Serial.printf("Servo angle set to %d\n", angle);
}
