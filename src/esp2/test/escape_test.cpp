#include "test_common_esp2.h"
#include <Arduino.h>
#include <cmath>

#include "common/data_types.h"
#include "EscapeController.h"

// MQTT topic for debug
const char* mqtt_topic_escape = "slamaleykoum77/escape_test";

// Global instance

uint32_t last_us = 0;

static float getYawIMU(const IMUData& imu_data) {
  float qw = imu_data.qw;
  float qx = imu_data.qx;
  float qy = imu_data.qy;
  float qz = imu_data.qz;
  float siny_cosp = 2.0 * (qw * qz + qx * qy);
  float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return atan2(siny_cosp, cosy_cosp);
}


float yaw0 = 0;
char bufcren[100];

void setup_escapeTest() {
    Serial.begin(115200);
    delay(1500);

    connection.setupWifi();
    connection.check_connection();

    snprintf(bufcren, sizeof(bufcren), "Setup Complete");
    connection.publish(mqtt_topic_escape, bufcren);


    I2C_wire.begin(8, 9);
    i2cMutexInit();

    // Init IMU
    if (!imu.begin()) {
        Serial.println("IMU init failed!");
        while (true);
    }

    // Init encoder
    if (!encoder.begin()) {
        Serial.println("Encoder init failed!");
        while (true);
    }

    // Init motor
    if (!motor.begin()) {
        Serial.println("Motor init failed!");
        while (true);
    }

    // Init steering servo
    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }

    servo_dir.setAngle(90); // center

    imu.readAndUpdate();
    yaw0 = getYawIMU(imu.data());

    // ----------------------------------------------------
    // 🚀 START ESCAPE IMMEDIATELY
    // ----------------------------------------------------
    escape.begin(yaw0);
    Serial.println("[ESCAPE TEST] Escape triggered immediately!");

    
    last_us = micros();

   
}

void loop_escapeTest() {
    // Compute dt
    uint32_t now = micros();
    float dt = (now - last_us) * 1e-6f;
    last_us = now;



    i2c_lock();
    imu.readAndUpdate();
    IMUData imuData = imu.data();
    i2c_unlock();

    float yaw = getYawIMU(imuData); // rad

    motor.forward(0.17f);
    motor.update();

    //Run escape behavior
    escape.update(yaw, dt);

    // Motor smoothing update
    motor.update();
}