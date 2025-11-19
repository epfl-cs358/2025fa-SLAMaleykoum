/**
 * @file EncoderCarVelocity.cpp
 * @brief Implementation of the EncoderCarVelocity class to be used to calculate the car's velocity
 * 
 * 
 * @author SLAMaleykoum
 * @date Nov 2025
 */
#include "EncoderCarVelocity.h"
#include <Wire.h>
#include "I2C_wire.h"

EncoderCarVelocity::EncoderCarVelocity() {}

// Initialize I2C and AS5600
bool EncoderCarVelocity::begin() {
     
  


    I2C_wire.begin(); 


    if (!as5600.begin()) {
        Serial.println("AS5600 not detected!");
        return false;
    }

    // Set clockwise rotation, small noise filter
    as5600.setDirection(AS5600_CLOCK_WISE);
    as5600.setHysteresis(2);     // suppress small noise
    as5600.setSlowFilter(2);     // optional, reduces jitter
   
    return true;
}

// Returns motor angular velocity in rad/sec
float EncoderCarVelocity::getMotorAngularVelocity() {
    // library handles delta / dt internally
    return as5600.getAngularSpeed(AS5600_MODE_RADIANS, true);
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



float EncoderCarVelocity::getWheelAngularVelocity() {
    return getFilteredAngularVelocity() / GEAR_RATIO;
}



float EncoderCarVelocity::getWheelLinearVelocity() {
    return getWheelAngularVelocity() * WHEEL_RADIUS;
}


int32_t EncoderCarVelocity::getCumulativePosition() {
    return as5600.getCumulativePosition();
}

// NEW: Update distance traveled (call this at ~50–100 Hz)
float EncoderCarVelocity::getDistance() {

    int32_t current = as5600.getCumulativePosition();
    int32_t deltaTicks = current - lastCumTicks;
    lastCumTicks = current;

    // Convert motor ticks → motor rotations
    float motorRot = (float)deltaTicks / 4096.0f;

    // Convert motor → wheel (gear reduction)
    float wheelRot = motorRot / GEAR_RATIO;

    // Distance = rotations × circumference
    wheelDistanceMeters += wheelRot * (2.0f * PI * WHEEL_RADIUS);

    return wheelDistanceMeters ;
}
