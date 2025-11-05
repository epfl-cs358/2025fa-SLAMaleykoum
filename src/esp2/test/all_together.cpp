/**
 * @file all_together.cpp
 * @brief Test all the components together.
 * @result The ultrasonic sensor cannot initialize..
 */
#include "test_common_esp2.h"

static int drivePower = 20;
static int angle = 90;
static const int stepDeg = 5;
static const int minAngle = 60;
static const int maxAngle = 120;

void setup_all_together() {
    
    Serial.begin(115200);
    delay(2000);

    // Initialize motor
    if (!motor.begin()) {
        Serial.println("Motor init failed!");
        while (true);
    }

    // Initialize servo
    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }
    servo_dir.setAngle(angle);

    // Initialize ultrasonic sensor
    // if (!ultrasonic.begin()) {
    //     Serial.println("Ultrasonic init failed!");
    //     while (true);
    // }

    i2cMutexInit();

    // Initialize imu
    if (!imu.begin()) {
        Serial.println("IMU init failed!");
        while(true);
    }

    // Initialize encoder
    if (!encoder.begin()) {
        Serial.println("encoder init failed!");
        while(true);
    }

    Serial.println("Use W/S to drive, SPACE to stop, A/D to steer, Q to quit.");
}

// test loop
void loop_all_together() {
    float dist = ultrasonic.readDistance();

    // test : if the car is moved up then stops
    IMUData data = imu.data();
    delay(10);
    if (data.acc_z > 0.2) {
        motor.stop();
    }

    // test the motor and the ultrasonic sensor
    if (dist < 0 || dist > UltraSonicSensor::STOP_THRESHOLD) {
        // make the motor and the wheels turn
        motor.forward(drivePower / 100.0f);
        Serial.println("I'm moving forward");
    } else {
        motor.stop();
        Serial.println("Obstacle! I stopped");
    }

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

    motor.update();
    delay(100);
    imu.readAndUpdate();
    encoder.update();
    delay(100);
}