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

// Initialize I2C and AS5600
bool EncoderCarVelocity::begin() {
    //Wire.begin();  // default SDA/SCL pins
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

/**
 * @brief Reads motor angular velocity directly from AS5600.
 * 
 * The AS5600 library internally calculates angular speed by tracking
 * angle changes over time. Update flag is set to true to force a fresh read.
 * 
 * @return Motor shaft angular velocity in radians per second.
 */
float EncoderCarVelocity::getMotorAngularVelocity() {
    // library handles delta / dt internally
    return as5600.getAngularSpeed(AS5600_MODE_RADIANS, true);
}

/**
 * @brief Returns filtered motor angular velocity using dual-stage filtering.
 * 
 * Filtering stages:
 * 1. Exponential Moving Average (EMA) with alpha=0.02 (98% historical weight)
 * 2. 5-sample moving average for additional smoothing
 * 
 * This provides stable velocity estimates with minimal lag.
 * 
 * @return Filtered motor angular velocity in radians per second.
 */
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


/**
 * @brief Converts motor angular velocity to wheel angular velocity.
 * 
 * Accounts for gear reduction between motor and wheel (GEAR_RATIO = 10:1).
 * 
 * @return Wheel angular velocity in radians per second.
 */
float EncoderCarVelocity::getWheelAngularVelocity() {
    return getMotorAngularVelocity()/GEAR_RATIO;// getFilteredAngularVelocity() / GEAR_RATIO;
}


/**
 * @brief Converts wheel angular velocity to linear velocity.
 * 
 * Uses the wheel radius to calculate the car's linear speed.
 * This represents the actual ground speed of the vehicle.
 * 
 * @return Linear velocity in meters per second.
 */
float EncoderCarVelocity::getWheelLinearVelocity() {
    return getWheelAngularVelocity() * WHEEL_RADIUS;
}

/**
 * @brief Returns raw cumulative position from AS5600 encoder.
 * 
 * The cumulative position is a 32-bit signed integer representing total
 * encoder rotations since initialization, including direction (±).
 * 
 * @return Cumulative encoder position in ticks (4096 ticks per revolution).
 */
int32_t EncoderCarVelocity::getCumulativePosition() {
    return as5600.getCumulativePosition();
}

/**
 * @brief Calculates and returns cumulative distance traveled.
 * 
 * Process:
 * 1. Reads current cumulative position from AS5600
 * 2. Calculates incremental ticks since last call
 * 3. Converts ticks -> motor rotations (4096 ticks/rev)
 * 4. Converts motor rotations -> wheel rotations (÷ gear ratio)
 * 5. Converts wheel rotations -> linear distance (× wheel circumference)
 * 6. Accumulates into total distance traveled
 * 
 * @note This function maintains state and accumulates over time.
 * @note Must be called at ≥100 Hz for accurate tracking (AS5600 requirement).
 * 
 * @return Total distance traveled in meters since begin() was called.
 */
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
