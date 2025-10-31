/**
 * @file servo_right_left.cpp
 * @brief Test to makes the servo of the wheels turn left and right
 */
#include "test_common.h"

static int angle = 90;
static const int stepDeg = 15;
static const int minAngle = 60;
static const int maxAngle = 120;

void setup_servo_right_left() {
    
    Serial.begin(115200);
    delay(2000);

    // Initialize servo
    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }
    servo_dir.setAngle(angle);

    Serial.println("Set up ready to test the servo!");
}

// test loop
void loop_servo_right_left() {
    // test the servo for the wheels direction (left)
    angle = max(minAngle, angle - stepDeg);
    servo_dir.setAngle(angle);
    Serial.print("Steer Left: "); 
    Serial.println(angle);

    delay(2000);

    // test the servo for the wheels direction (right)
    angle = min(maxAngle, angle + stepDeg);
    servo_dir.setAngle(angle);
    Serial.print("Steer Right: "); 
    Serial.println(angle);

    delay(2000);
}
