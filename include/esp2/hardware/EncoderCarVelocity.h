/**
 * @file EncoderCarVelocity.h
 * @brief Interface for measuring car velocity and position using AS5600 magnetic encoder.
 * 
 * This class provides methods to read motor and wheel angular/linear velocities,
 * as well as cumulative distance traveled. It interfaces with the AS5600 encoder
 * via I2C and accounts for gear ratio and wheel dimensions.
 * 
 * @note Requires the AS5600 library (https://github.com/RobTillaart/AS5600) and I2C_wire configuration.
 * @note getDistance() must be called at 100+ Hz for accurate cumulative position tracking.
 * 
 * @author SLAMaleykoum 
 * @date Nov 2025
 */

#ifndef ENCODER_CAR_VELOCITY_H
#define ENCODER_CAR_VELOCITY_H

#include <AS5600.h>
#include "I2C_wire.h"

#define GEAR_RATIO 10.0f     // Motor to wheel gear reduction ratio (10:1) 
#define WHEEL_RADIUS 0.0495f // Wheel radius in meters //maybe needs to be adapted but 9.9 diametre  

/**
 * @class EncoderCarVelocity
 * @brief Manages velocity and position measurements from AS5600 magnetic encoder.
 * 
 * Provides real-time measurements of motor/wheel speeds and cumulative distance.
 * Uses AS5600's cumulative position tracking for accurate odometry.
 */
class EncoderCarVelocity {
public:
    /**
     * @brief Constructs an EncoderCarVelocity object.
     */
    EncoderCarVelocity();

    /**
     * @brief Initializes the AS5600 encoder and I2C communication.
     * 
     * Configures the encoder for clockwise rotation with noise filtering.
     * Must be called before using any measurement functions.
     * 
     * @return true if encoder is detected and initialized successfully, false otherwise.
     */
    bool begin();  

    /**
     * @brief Gets the motor shaft angular velocity.
     * 
     * Reads the instantaneous angular speed directly from the AS5600.
     * 
     * @return Motor angular velocity in radians per second.
     * 
     * @note library handles delta / dt internally
     */
    float getMotorAngularVelocity() {return as5600.getAngularSpeed(AS5600_MODE_RADIANS, true);}
    

    /**
     * @brief Gets the wheel angular velocity (accounts for gear ratio).
     * 
     * Converts motor speed to wheel speed using GEAR_RATIO.
     * 
     * @return Wheel angular velocity in radians per second.
     */
    float getWheelAngularVelocity() {return getMotorAngularVelocity() / GEAR_RATIO;} 
    
    /**
     * @brief Gets the linear velocity of the car.
     * 
     * Converts wheel angular velocity to linear velocity using wheel radius.
     * This is the actual speed at which the car is moving.
     * 
     * @return Linear velocity in meters per second.
     */
    float getWheelLinearVelocity() {return getWheelAngularVelocity() * WHEEL_RADIUS;}
    
    /**
     * @brief Gets the raw cumulative position from AS5600.
     * 
     * Returns the total encoder ticks accumulated since initialization.
     * Used internally by getDistance() for precise position tracking.
     * 
     * @return Cumulative position in encoder ticks (int32_t).
     */
    int32_t getCumulativePosition() {return as5600.getCumulativePosition();}

    /**
     * @brief Calculates the total distance traveled by the car.
     * 
     * Accumulates distance based on encoder tick changes between calls.
     * Returns cumulative distance since begin() was called.
     * 
     * Process:
     * 1. Reads current cumulative position from AS5600
     * 2. Calculates incremental ticks since last call
     * 3. Converts ticks -> motor rotations (4096 ticks/rev)
     * 4. Converts motor rotations -> wheel rotations (÷ gear ratio)
     * 5. Converts wheel rotations -> linear distance (× wheel circumference)
     * 6. Accumulates into total distance traveled
     * 
     * @note Must be called at 100+ Hz for accurate tracking per AS5600 requirements.
     * @note This function maintains internal state and accumulates over time.
     * 
     * @return Total distance traveled in meters since begin() was called (cumulative).
     */
    float getDistance();

private:
    AS5600 as5600 = AS5600(&I2C_wire); // AS5600 encoder instance

    int32_t lastCumTicks = 0; // Previous cumulative tick count
    float wheelDistanceMeters= 0.0f;  // Accumulated distance in meters;
};

#endif
