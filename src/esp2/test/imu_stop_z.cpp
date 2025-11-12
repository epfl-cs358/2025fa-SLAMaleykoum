/**
 * @file imu_stop_z.cpp
 * @brief Test to make the motor stop when the car is quickly moved up. This tests the 
 * received data from the imu.
 */
#include "test_common_esp2.h"

static int drivePower = 20;
const char* mqtt_topic_calibration2 = "slamaleykoum77/imu";

void setup_imu_stop_z() {
    
    Serial.begin(115200);
    delay(2000);

    connection.setupWifi();

    // Initialize motor
    if (!motor.begin()) {
        Serial.println("Motor init failed!");
        while (true);
    }

    i2cMutexInit();

    // Initialize imu
    if (!imu.begin()) {
        Serial.println("IMU init failed!");
        while(true);
    }

    Serial.println("Set up ready to test the imu!");
}

// test loop
void loop_imu_stop_z() {
    connection.check_connection();
    // test : if the car is moved up then stops
    IMUData data = imu.data();
    delay(100);
    if (data.acc_z > 0.2) {
        motor.stop();
    }

    // make the motor and the wheels turn
    motor.forward(drivePower / 100.0f);
    Serial.println("I'm moving forward");
    char msg[60];
    snprintf(msg, sizeof(msg), "Acc= %0.3f", data.acc_z);
    // Publish data to MQTT
    connection.publish(mqtt_topic_calibration2, msg);

    motor.update();
    delay(150);
    imu.readAndUpdate();
}