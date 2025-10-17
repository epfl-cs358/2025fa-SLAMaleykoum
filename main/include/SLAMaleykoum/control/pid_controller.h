/**
 * @file pid_controller.h
 * @brief Generic PID controller utility.
 */

#pragma once
#include <algorithm>

namespace SLAMaleykoum {
namespace control {

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    void reset();
    float compute(float setpoint, float measurement, float dt);

private:
    float kp_;
    float ki_;
    float kd_;

    float prev_error_{0.0f};
    float integral_{0.0f};
};

}  // namespace control
}  // namespace SLAMaleykoum
