/**
 * @file test_common.h
 * @brief Declares global hardware objects, pin mappings, and shared test function prototypes.
 *
 * This header centralizes all common definitions, hardware object declarations, 
 * and test function prototypes used across the various test modules.
 *
 * @details
 * - Defines all pin mappings for the ESP32-S3 board used in the project.  
 * - Declares all global hardware interfaces (motor, servo, ultrasonic sensor, IMU, encoder).  
 * - Declares the global `Connection` object for Wi-Fi and MQTT communication.  
 * - Provides prototypes for each `setup_*()` and `loop_*()` function, 
 *   allowing `test_main.cpp` to call the corresponding tests dynamically.
 *
 * @note
 * The global objects are **defined** once in `test_globals.cpp`.  
 * This file only provides `extern` declarations to make them accessible 
 * across all test files.
 */
#pragma once
#include <Arduino.h>
#include "MotorManager.h"
#include "DMS15.h"
#include "UltraSonicSensor.h"
#include "ImuSensor.h"
#include "I2C_mutex.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "common/wifi_connection.h"
#include "EncoderCarVelocity.h"
#include "MotorController.h"
#include "AS5600.h"
#include "motor_pid.h"
#include "../../include/common/esp_link.h"
#include <odometry.h>
#include "esp2/control/pure_pursuit.h"

// Pins used by the hardware components on the esp 2
#define ESC_PIN 15          // pin used for the motor
#define SERVO_DIR_PIN 6     // the servo that modifies the direction of the wheels
#define US_TRIG_PIN 5       // changé par rapport a avant sur conseil de chat
#define US_ECHO_PIN 19      // changé par rapport a avant sur conseil de chat
#define  SDA_PIN 8           // pin used for SDA of IMU
#define SCL_PIN 9           // pin used for SCL of IMU

// Harwdare objects
extern MotorManager motor;
//extern MotorController motor;
extern DMS15 servo_dir;
extern UltraSonicSensor ultrasonic;
extern ImuSensor imu; 
//extern AS5600Encoder encoder;
extern EncoderCarVelocity encoder;
extern Connection connection;
extern HardwareSerial ESPS;

extern MotorPID pid;

extern MotorPID pid;
extern PurePursuit purePursuit;
extern Odometry odom;

// Prototypes of the functions
void setup_all_together();          void loop_all_together();
void setup_imu_stop_z();            void loop_imu_stop_z();
void setup_motor_start_stop();      void loop_motor_start_stop();
void setup_servo_right_left();      void loop_servo_right_left();
void setup_motor_runs();            void loop_motor_runs();
void setup_imu_calibration();       void loop_imu_calibration();
void setup_encoder();               void loop_encoder();
void setup_test_connection();       void loop_test_connection();
void setup_pid_velocity();          void loop_pid_velocity();
void setup_path_pid();              void loop_path_pid();
void setup_test_freertos_path_follow();  void loop_test_freertos_path_follow();
void setup_test_sensor();           void loop_test_sensor();
void setup_esps_comm_esp2();        void loop_esps_comm_esp2();
void setup_led_basic();             void loop_led_basic();
