/**
 * @file test_common.h 
 * @brief Regroups the common variables and the setup and loop functions
 */
#pragma once
#include <Arduino.h>
#include "MotorManager.h"
#include "DMS15.h"
#include "UltraSonicSensor.h"
#include "ImuSensor.h"
#include "I2C_mutex.h"
#include "AS5600Encoder.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "wifi_connection.h"

// Pins used by the hardware components on the esp 2
#define ESC_PIN 15          // pin used for the motor
#define SERVO_DIR_PIN 6     // the servo that modifies the direction of the wheels
#define US_TRIG_PIN 5       // changé par rapport a avant sur conseil de chat
#define US_ECHO_PIN 19      // changé par rapport a avant sur conseil de chat
#define SDA_PIN 8           // pin used for SDA of IMU
#define SCL_PIN 9           // pin used for SCL of IMU

// Harwdare objects
extern MotorManager motor;
extern DMS15 servo_dir;
extern UltraSonicSensor ultrasonic;
extern ImuSensor imu; 
extern AS5600Encoder encoder;

extern Connection connection;

// Prototypes of the functions
void setup_all_together();          void loop_all_together();
void setup_imu_stop_z();            void loop_imu_stop_z();
void setup_motor_start_stop();      void loop_motor_start_stop();
void setup_servo_right_left();      void loop_servo_right_left();
void setup_motor_runs();            void loop_motor_runs();
void setup_encoder();               void loop_encoder();
void setup_test_connection();       void loop_test_connection();
