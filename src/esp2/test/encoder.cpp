#include "test_common.h"

static int drivePower = 20;

void setup_encoder() {
    Serial.begin(115200);
    delay(2000);

    // Initialize motor
    if (!motor.begin()) {
        Serial.println("Motor init failed!");
        while (true);
    }

    i2cMutexInit();

    // Initialize encoder
    if (!encoder.begin()) {
        Serial.println("encoder init failed!");
        while(true);
    }
}

void loop_encoder() {

    uint16_t angle = encoder.getRawAngle();

    motor.forward();
    delay(10);
    motor.update();
    delay(10);
    encoder.update();
    delay(10);

}