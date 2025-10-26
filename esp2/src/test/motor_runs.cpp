/**
 * @file motor_runs
 * @brief Test to make the motor run simply
 */
#include "test_common.h"

static int drivePower = 20;

void setup_motor_runs() {
    
    Serial.begin(115200);
    delay(2000);

    // Initialize motor
    if (!motor.begin()) {
        Serial.println("Motor init failed!");
        while (true);
    }

    Serial.println("Set up ready to test the motor!");
}

// test loop
void loop_motor_runs() {
    // make the motor and the wheels turn
    motor.forward(drivePower / 100.0f);
    motor.update();
}