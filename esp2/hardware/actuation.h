// Filename: esp2/hardware/actuation.h
// Description: Contract for low-level interaction with motor PWM and encoders.


#pragma once

#include "common/data_types.h"
#include <stdint.h>

/**
 * @brief Handles low-level interaction with hardware components (motors/encoders/IMU).
 * This module is designed to be mocked for TDD of the PID/EKF algorithms.
 */
class Actuation {
public:
    Actuation();

    /**
     * @brief Reads the latest raw wheel encoder ticks and converts to distance change.
     */
    OdometryData read_odometry();

    /**
     * @brief Reads the latest raw IMU data.
     */
    IMUData read_imu();

    /**
     * @brief Applies the PWM values directly to the motor drivers.
     */
    void set_motor_pwm(const MotorController::MotorOutputs& outputs);

    /**
     * @brief Stops all motors immediately.
     */
    void emergency_stop();

private:
    // Hardware-specific register/pin settings
};
