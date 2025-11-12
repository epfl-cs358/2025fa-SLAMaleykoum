/**
 * @file    servo.cpp
 * @author  2025sp-turboslam (last year's team)
 * @brief   Source file for DMS15 servo motor control class.
 * @version XX.X
 * @date    XX-XX-2025
 *
 *
 * @details
 *  This module provides public functions for initializing and
 *  controlling a DMS15 servo that tilts the LiDAR sensor.
 *  It supports both smooth oscillation and manual control.
 */


#include "servo.h"
#include <Arduino.h>

// Servo constructor => create an instance of the DMS15 servo class
DMS15::DMS15(int pin) : servoPin(pin) {}

/** Initialises servo, returns true if it's ready to move. */
bool DMS15::begin() {
    servo.attach(servoPin);
    Serial.printf("Servo attached to pin %d\n", servoPin);
    servo.write(90); 
    return servo.attached(); 
}

/*
 Function to sweep lidar between angleMin and angleMax over a period T_ms
 example: lidarservo.tiltLidar(70, 10, 2000) -> Tilt between 70° and 10° in 2sec oscillations.
*/
void DMS15::tiltLidar(float angleMin, float angleMax, unsigned long T_ms) {
    unsigned long now = millis();
    float t = fmod(now, T_ms) / (float)T_ms;

    float progress = (t < 0.5f) ? (t * 2.0f) : (2.0f * (1.0f - t));
    float angle = angleMin + (angleMax - angleMin) * progress;

    currentAngle = angle;
    servo.write(angle);
}


// OLD CODE ?
// -----------------------------------------------------------
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
// -----------------------------------------------------------


/*
 Manually set servo to specific angle between 0 and 180 degrees
 (We will use this to block the lidar for 2d mapping)
*/
void DMS15::setAngle(int angle) {
    angle = constrain(angle, 0, 180); 
    servo.write(angle);
    Serial.printf("Servo angle set to %d\n", angle);
}

// Helper function to get current angle of the servo
int DMS15::getAngle() const {
    return currentAngle;
}