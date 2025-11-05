/**
 * @file EncoderCarVelocity.cpp
 * @brief Implementation of the EncoderCarVelocity class to be used to calculate the car's velocity
 * 
 * 
 * @author SLAMaleykoum
 * @date Nov 2025
 */
#include "EncoderCarVelocity.h"
#include "AS5600Encoder.h"
#include "I2C_mutex.h"
#include "I2C_wire.h"


EncoderCarVelocity::EncoderCarVelocity(AS5600Encoder* encoder)
    : encoder(encoder),
      lastAngle(0.0f),
      lastTime(0),
      motorAngularVelocity(0.0f) {} //might not need later, for testing

   
void EncoderCarVelocity::update(unsigned long currentMillis) {
    float newAngle = encoder->update(); 
    if (newAngle < 0.0f) return;         // encoder read error

    if (lastTime == 0) {
        // first sample: initialize
        lastAngle = newAngle;
        unwrappedAngle = newAngle;
        lastTime = currentMillis;
        return;
    }

    unsigned long dt_ms = currentMillis - lastTime;
    if (dt_ms == 0) return;

    // --- unwrap angle into continuous degrees ---
    float delta = newAngle - lastAngle;

    // if we jumped across the 0/360 boundary, correct it
    if (delta > 180.0f)      delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;

    unwrappedAngle += delta;   // accumulate continuous angle

    // --- compute velocity from delta ---
    float dt = dt_ms / 1000.0f;
    float angularVel_rad_s = (delta * DEG_TO_RAD) / dt;

    // optional smoothing
    const float alpha = 0.2f;
    motorAngularVelocity =
        (motorAngularVelocity == 0.0f)
            ? angularVel_rad_s
            : alpha * angularVel_rad_s + (1 - alpha) * motorAngularVelocity;

    // update state for next call
    lastAngle = newAngle;
    lastTime  = currentMillis;
}

float EncoderCarVelocity::getMotorAngularVelocity() const {
    return motorAngularVelocity;
}

float EncoderCarVelocity::getWheelAngularVelocity() const {
    return motorAngularVelocity / GEAR_RATIO;
}

float EncoderCarVelocity::getWheelLinearVelocity() const {
    return getWheelAngularVelocity() * WHEEL_RADIUS;
}

float EncoderCarVelocity::getLastAngle() const {
    return lastAngle;
}