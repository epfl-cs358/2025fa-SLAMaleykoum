// Filename: esp2/control/motor_pid.h
// Description: Contract for the high-frequency PID motor control loop on ESP_2.

#pragma once

#include "common/data_types.h"
#include <stdint.h>

// TODO: Define `MotorOutputs` type.

/**
 * @brief Manages the closed-loop PID control of the drive motors.
 * Runs in the highest priority task (100-200 Hz).
 */
class MotorPID {
public:
    /**
     * @brief Initializes the Motor Controller with PID gains.
     * @param Kp, Ki, Kd PID gains for the velocity loop.
     */
    MotorPID(float Kp, float Ki, float Kd);

    /**
     * @brief Calculates motor PWM based on target and current velocities.
     * @param target_velocity The desired velocity from Pure Pursuit.
     * @param current_velocity The current estimated velocity (from Localization EKF).
     * @return The final MotorOutputs (PWM values).
     */
    MotorOutputs compute_motor_commands(const Velocity& target_velocity, const Velocity& current_velocity);

    /**
     * @brief Directly applies the calculated PWM values to the motor hardware pins.
     */
    void apply_outputs(const MotorOutputs& outputs);

    /**
     * @brief Immediately stops the motors.
     */
    void emergency_stop();

private:
    float Kp_, Ki_, Kd_;
    // Internal state variables for PID
};
