// Filename: esp2/control/motor_pid.cpp
// Description: Implementation of the high-frequency PID motor control loop on ESP_2.

// Note: our car only has one motor, so our pid will have as input the target
//      and current velocity of the car, and output the pwm value for the motor.
//      To check the current velocity of the car we use the odometry data from
//      the motor enocoder, and to get the target velocity we use the output from
//      the pure pursuit controller.

//      The motor controller runs in the highest priority task (100-200 Hz).

// For manual testing, we can set the target velocity to a constant value
//      and see how well the PID controller maintains that velocity.
//      If we set the target velocity for a fixed time we can check the distance
//      traveled by the car to verify the velocity control. By setting the target
//      velocity to different values we can also test the responsiveness of the
//      controller.
//      If we set the target velocity to zero, we can test the braking performance
//      of the car. 

// NOTE that the breaking performance could potentially be better if instead of
//      just setting the motor pwm to zero, we actively reverse the motor in bursts
//      to decelerate faster. This could be implemented in the emergency_stop()
//      function.


#include "motor_pid.h"

MotorPID::MotorPID(float Kp, float Ki, float Kd)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd) {
    reset();
}

void MotorPID::reset() {
    integral_ = 0.0f;
    prev_err_ = 0.0f;
}

MotorOutputs MotorPID::compute_pwm_output(const Velocity& target_vel, const Velocity& current_velocity, float dt) {
    
    // Prevent division by zero or negative time
    if (dt <= 0.000001f) {
        return NEUTRAL_US; // Return neutral if time has not advanced
    }

    // Calculate Error
    float e = target_vel.v_linear - current_velocity.v_linear;

    // Calculate Integral (with anti-windup)
    integral_ += e * dt;
    integral_ = constrain(integral_, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);

    // Calculate Derivative
    float derivative = (e - prev_err_) / dt;

    // Compute full PID control signal (scaled -1.0 to 1.0)
    float control_signal = (Kp_ * e) + (Ki_ * integral_) + (Kd_ * derivative);

    // Store error for next iteration
    prev_err_ = e;

    // Clamp the control signal
    return constrain(control_signal, PID_CONTROL_MIN, PID_CONTROL_MAX);

}