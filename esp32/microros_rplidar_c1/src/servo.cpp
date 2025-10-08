#include "DMS15.h"
#include <Arduino.h>

DMS15::DMS15(int pin) : servoPin(pin) {}

bool DMS15::begin() {
    servo.attach(servoPin);
    Serial.printf("Servo attached to pin %d\n", servoPin);
    servo.write(110); 
    return servo.attached(); 
}

void DMS15::tiltLidar(float angleMin, float angleMax, unsigned long T_ms) {
    unsigned long now = millis();
    float t = fmod(now, T_ms) / (float)T_ms;

    float progress = (t < 0.5f) ? (t * 2.0f) : (2.0f * (1.0f - t));
    float angle = angleMin + (angleMax - angleMin) * progress;

    currentAngle = angle;
    servo.write(angle);
}

// void DMS15::tiltLidar() {
//     static bool tiltUp = false;
//     static unsigned long lastMoveTime = 0;
//     const unsigned long moveInterval = 600;  // 1 second

//     unsigned long currentTime = millis();
//     if (currentTime - lastMoveTime >= moveInterval) {
//         if (tiltUp) {
//             servo.write(110);
//         } else {
//             servo.write(70);
//         }
//         tiltUp = !tiltUp;
//         lastMoveTime = currentTime;
//     }
// }

void DMS15::setAngle(int angle) {
    angle = constrain(angle, 0, 180); 
    servo.write(angle);
    Serial.printf("Servo angle set to %d\n", angle);
}

int DMS15::getAngle() const {
    return currentAngle;
}




