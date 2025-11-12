/**
 * @file main.cpp
 * @brief Main control program for the SLAMaleykoum autonomous car.
 *
 * This file contains the primary firmware logic executed on the ESP32-S3 board.  
 * It integrates all hardware modules — motor, servo, ultrasonic sensor, IMU, and encoder — 
 * as well as the Wi-Fi and MQTT connection system.  
 * The current implementation serves as a comprehensive integration test, 
 * which will later evolve into the final autonomous driving program.
 *
 * @note
 * This version is primarily designed for validation and debugging.  
 * The final release will include autonomous navigation logic, data fusion, 
 * and closed-loop control algorithms.
 *
 * @author SLAMaleykoum
 * @date October 2025
 */
#include <Arduino.h>
#include "MotorManager.h"
#include "DMS15.h"
#include "UltraSonicSensor.h"
#include "ImuSensor.h"
#include "I2C_mutex.h"
#include "AS5600Encoder.h"
#include "common/wifi_connection.h"

// Pins used by the hardware components on the esp 2
#define ESC_PIN 15          // pin used for the motor
#define SERVO_DIR_PIN 6     // the servo that modifies the direction of the wheels
#define US_TRIG_PIN 5       // pin used for TRIG of sensor
#define US_ECHO_PIN 19      // pin used for ECHO of sensor
#define SDA_PIN 8           // pin used for SDA of IMU
#define SCL_PIN 9           // pin used for SCL of IMU

Connection connection;      // variable for the connection between esp and computer

// Harwdare objects 
MotorManager motor(ESC_PIN);
DMS15 servo_dir(SERVO_DIR_PIN);
UltraSonicSensor ultrasonic(US_TRIG_PIN, US_ECHO_PIN);
ImuSensor imu(SDA_PIN, SCL_PIN);
AS5600Encoder encoder(SDA_PIN, SCL_PIN);

// Parameters
int drivePower = 20;
int angle = 90;
const int stepDeg = 5;
const int minAngle = 60;
const int maxAngle = 120;

const char* mqtt_topic = "slamaleykoum77/ultrasonic";

void setup() {
    
    Serial.begin(115200);

    connection.setupWifi();

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
    if (!ultrasonic.begin()) {
        Serial.println("Ultrasonic init failed!");
        while (true);
    }

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

    Serial.println("Setup ready!");
}

void loop() {
    connection.check_connection();

    // Print the current state at the beginning of the loop
    Serial.println(imu.data());

    // Print the current motor angle at the beginning of the loop
    Serial.print("Motor angle = ");
    Serial.println(encoder.update());

    float dist = ultrasonic.readDistance(); //read once per loop so no multiple actions at the same time

    char msg[50];
    snprintf(msg, sizeof(msg), "Distance: %.2f cm", dist);
    // Publish data to MQTT
    connection.publish(mqtt_topic, msg);

    motor.forward(drivePower / 100.);
    // Emergency stop if an obstacle is detected
    if (dist > 0 && dist < UltraSonicSensor::STOP_THRESHOLD) {
        motor.stop();
        Serial.println("Emergency Stop! Obstacle detected.");
    }

    motor.update();
    delay(100);
    imu.readAndUpdate();
    encoder.update();
    delay(100);
}