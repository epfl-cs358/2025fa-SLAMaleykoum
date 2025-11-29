#include "test_common_esp1.h"

void setup_servo_lidar() {
    
    Serial.begin(115200);
    delay(2000);

    // Initialize servo
    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }
    //servo_dir.setAngle(angle);

    Serial.println("Set up ready to test the servo!");
}

// test loop
void loop_servo_lidar() {
    //servo_dir.setAngle(15);

    delay(2000);
}
