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


EncoderCarVelocity::EncoderCarVelocity() {}

// Initialize I2C and AS5600
void EncoderCarVelocity::begin() {
    Wire.begin();  // default SDA/SCL pins
    if (!as5600.begin()) {
        Serial.println("AS5600 not detected!");
        while (1); 
    }

    // Set clockwise rotation, small noise filter
    as5600.setDirection(AS5600_CLOCK_WISE);
    as5600.setHysteresis(2);     // suppress small noise
    as5600.setSlowFilter(2);     // optional, reduces jitter
   
}

// Returns motor angular velocity in rad/sec
float EncoderCarVelocity::getMotorAngularVelocity() {
    // library handles delta / dt internally
    return as5600.getAngularSpeed(AS5600_MODE_RADIANS, true);
}


float EncoderCarVelocity::getFilteredAngularVelocity() {
    /*static float filteredVel = 0.0f;
    const float alpha = 0.02f;

    float rawVel = getMotorAngularVelocity();
    filteredVel = alpha * rawVel + (1 - alpha) * filteredVel;

    return filteredVel;*/
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
