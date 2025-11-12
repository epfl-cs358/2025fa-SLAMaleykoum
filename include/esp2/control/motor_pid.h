// Filename: esp2/control/motor_pid.h
// Description: Takes a target velocity and current velocity, and computes
//      the required PWM pulse width (microseconds) for the motor controller.

#pragma once

#include "common/data_types.h" // For Velocity and MotorOutputs types
#include "MotorController.h"   // For NEUTRAL_US, MAX_FORWARD_US, MAX_REVERSE_US
#include <Arduino.h>           // For constrain()

// --- Tuning Parameters ---
// These limits prevent the integral term from
// growing too large (integral windup).
// You will need to tune these values.
#define PID_INTEGRAL_MIN -5.0f
#define PID_INTEGRAL_MAX 5.0f

// These limits clamp the intermediate control signal
// before it's scaled to PWM values.
#define PID_CONTROL_MIN -1.0f
#define PID_CONTROL_MAX 1.0f

class MotorPID {
public:
    /**
     * @brief Constructor to initialize PID gains.
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    MotorPID(float Kp, float Ki, float Kd);

    /**
     * @brief Computes the raw PWM motor command.
     * @param target_vel The desired velocity (from Pure Pursuit).
     * @param current_velocity The measured velocity (from EKF/Odometry).
     * @param dt The time delta since the last compute call (in seconds).
     * @return The raw PWM pulse width (MotorOutputs type, e.g., 1000-2000µs)
     */
    MotorOutputs compute_pwm_output(const Velocity& target_vel, const Velocity& current_velocity, float dt);

    /**
     * @brief Resets the integral and derivative states.
     * Call this when disabling the PID or after a long period of disuse.
     */
    void reset();

private:
    // PID Gains
    float Kp_;
    float Ki_;
    float Kd_;

    // PID State
    float integral_;
    float prev_err_;
};
