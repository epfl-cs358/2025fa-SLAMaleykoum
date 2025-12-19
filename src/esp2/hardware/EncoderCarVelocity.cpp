/**
 * @file EncoderCarVelocity.cpp
 * @brief Implementation of the EncoderCarVelocity class for velocity and position tracking.
 * 
 * Provides methods to measure motor/wheel speeds and cumulative distance traveled
 * using the AS5600 magnetic encoder. Handles gear ratio conversion and wheel kinematics.
 * 
 * @author SLAMaleykoum
 * @date Nov 2025
 */
#include "hardware/EncoderCarVelocity.h"
#include <Wire.h>
#include "hardware/I2C_wire.h"

//TODO add all the protections for nana here and odometry!!
EncoderCarVelocity::EncoderCarVelocity() {}

bool EncoderCarVelocity::begin() {
    //Wire.begin();  // default SDA/SCL pins
    if (!as5600.begin()) {
        Serial.println("AS5600 not detected!");
        return false;
    }

    as5600.setDirection(AS5600_CLOCK_WISE);
    as5600.setHysteresis(2);     // suppress small noise
    as5600.setSlowFilter(2);     // optional, reduces jitter
   
    return true;
}

float EncoderCarVelocity::getFilteredAngularVelocity() {

    static float emaVel = 0.0f;
    static float buffer[5] = {0};
    static int idx = 0;

    const float alpha = 0.02f;

    // Update EMA
    float rawVel = getMotorAngularVelocity();
    emaVel = alpha * rawVel + (1 - alpha) * emaVel;

    // Update buffer
    buffer[idx] = emaVel;
    idx = (idx + 1) % 5;


    // Compute small moving average
    float sum = 0;
    for (int i = 0; i < 5; i++) sum += buffer[i];


    return sum / 5.0f;
}

float EncoderCarVelocity::getDistance() {

    int32_t current = as5600.getCumulativePosition();
    int32_t deltaTicks = current - lastCumTicks;
    lastCumTicks = current;

    // Convert motor ticks -> motor rotations
    float motorRot = (float)deltaTicks / 4096.0f;

    // Convert motor -> wheel (gear reduction)
    float wheelRot = motorRot / GEAR_RATIO;

    // Distance = rotations × circumference
    wheelDistanceMeters += wheelRot * (2.0f * PI * WHEEL_RADIUS);

    return wheelDistanceMeters ;
}
