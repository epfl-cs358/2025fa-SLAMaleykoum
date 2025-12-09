#pragma once

#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include <Arduino.h>

class EscapeController {
public:
    EscapeController(MotorManager& m, DMS15& s);

    void begin(float startYaw);
    void update(float yaw, float dt);
    bool isActive() const { return state != IDLE; }

private:
    enum State {
        IDLE,
        TURN_RIGHT_FORWARD,
        NEUTRAL_BEFORE_REVERSE,
        TURN_LEFT_BACKWARD,
        CHECK_ANGLE,
        DONE
    };

    State state = IDLE;

    MotorManager& motor;
    DMS15& steering;

    float initialYaw = 0.0f;
    float timer = 0.0f;
    float neutralTimer = 0.0f;

    float wrapAngle(float a);
};
