#include "EscapeController.h"
#include <cmath>

#define STEP_TIME          1.5f     // Duration of forward/backward movement
#define FORWARD_SPEED      0.17f
#define BACKWARD_SPEED     0.21f

#define NEUTRAL_HOLD_TIME  1.5f     // ESC needs neutral before reverse

#define ESCAPE_TARGET_ANGLE_RAD   (M_PI)  // 180 degrees

// Servo steer angles
#define ANGLE_CENTER   90
#define ANGLE_RIGHT    60
#define ANGLE_LEFT     120

EscapeController::EscapeController(MotorManager& m, DMS15& s)
    : motor(m), steering(s) {}

float EscapeController::wrapAngle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

void EscapeController::begin(float startYaw) {
    if (state != IDLE) return;

    initialYaw = startYaw;
    timer = STEP_TIME;
    state = TURN_RIGHT_FORWARD;

    Serial.println("[ESCAPE] Begin escape maneuver");
}

void EscapeController::update(float yaw, float dt) {
    if (state == IDLE)
        return;

    switch (state) {

    // -------------------------------------------------------------
    // 1) TURN RIGHT + FORWARD
    // -------------------------------------------------------------
    case TURN_RIGHT_FORWARD:
        steering.setAngle(ANGLE_RIGHT);
        motor.forward(FORWARD_SPEED);

        timer -= dt;
        if (timer <= 0.0f) {
            timer = 0.0f;

            // Stop before reversing
            motor.stop();
            neutralTimer = NEUTRAL_HOLD_TIME;

            state = NEUTRAL_BEFORE_REVERSE;
            Serial.println("[ESCAPE] Enter neutral before reverse");
        }
        break;

    // -------------------------------------------------------------
    // 2) HOLD NEUTRAL BEFORE BACKWARD (ESC REQUIREMENT)
    // -------------------------------------------------------------
    case NEUTRAL_BEFORE_REVERSE:
        motor.stop();  // ensure clean neutral output
        neutralTimer -= dt;

        if (neutralTimer <= 0.0f) {
            neutralTimer = 0.0f;

            timer = STEP_TIME;
            state = TURN_LEFT_BACKWARD;

            Serial.println("[ESCAPE] Enter backward movement");
        }
        break;

    // -------------------------------------------------------------
    // 3) TURN LEFT + BACKWARD
    // -------------------------------------------------------------
    case TURN_LEFT_BACKWARD:
        steering.setAngle(ANGLE_LEFT);
        motor.backward(BACKWARD_SPEED);

        timer -= dt;
        if (timer <= 0.0f) {
            timer = 0.0f;

            motor.stop();
            state = CHECK_ANGLE;

            Serial.println("[ESCAPE] Checking angle...");
        }
        break;

    // -------------------------------------------------------------
    // 4) CHECK ROTATION PROGRESS
    // -------------------------------------------------------------
    case CHECK_ANGLE: {
        float turned = fabsf(wrapAngle(yaw - initialYaw));

        Serial.printf("[ESCAPE] Turned: %.1f° / %.1f°\n",
                      turned * 180.0f / M_PI,
                      ESCAPE_TARGET_ANGLE_RAD * 180.0f / M_PI);

        if (turned >= ESCAPE_TARGET_ANGLE_RAD) {
            state = DONE;
        } else {
            // Reset cycle
            timer = STEP_TIME;
            state = TURN_RIGHT_FORWARD;
            Serial.println("[ESCAPE] Another escape cycle...");
        }
        break;
    }

    // -------------------------------------------------------------
    // 5) DONE
    // -------------------------------------------------------------
    case DONE:
        steering.setAngle(ANGLE_CENTER);
        motor.stop();
        state = IDLE;
        Serial.println("[ESCAPE] Escape maneuver complete!");
        break;

    case IDLE:
    default:
        break;
    }
}
