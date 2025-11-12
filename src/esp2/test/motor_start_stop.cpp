/**
 * @file motor_start_stop.cpp
 * @brief Test to make the motor runs then stop
 * @result The forward function do not work this way (no movement is observable)
 */
#include "test_common_esp2.h"

static int drivePower = 20;

void setup_motor_start_stop() {
    
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
void loop_motor_start_stop() {
    // make the motor and the wheels turn
    motor.forward(drivePower / 100.0f);
    delay(10);
    motor.update();
    Serial.println("I'm moving forward");

    delay(30000);

    motor.stop();
    delay(10);
    motor.update();
    Serial.println("I stop");

    delay(10000);
}