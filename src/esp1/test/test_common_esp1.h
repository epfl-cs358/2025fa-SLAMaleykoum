/**
 * @file test_common_esp1.h
 * @brief Declares global hardware objects, pin mappings, and shared test function prototypes.
 *
 * This header centralizes all common definitions, hardware object declarations, 
 * and test function prototypes used across the various test modules.
 *
 * @details
 * - Defines all pin mappings for the ESP32-S3 1 board used in the project.  
 * - Declares all global hardware interfaces (lidar).  
 * - Declares the global `Connection` object for Wi-Fi and MQTT communication.  
 * - Provides prototypes for each `setup_*()` and `loop_*()` function, 
 *   allowing `test_main_esp1.cpp` to call the corresponding tests dynamically.
 *
 * @note
 * The global objects are **defined** once in `test_globals.cpp`.  
 * This file only provides `extern` declarations to make them accessible 
 * across all test files.
 */
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "common/wifi_connection.h"
#include "esp1/hardware/lidar.h"
#include "../../include/common/esp_link.h"
#include "esp1/mapping/occupancy/bayesian_grid.h"
#include "esp_wifi.h"


// Pins used by the hardware components on the esp 1
constexpr int MAX_RANGE = 8000; // Maximum range for LIDAR in mm
constexpr int LIDAR_TIMEOUT_MS = 5000; // Timeout for LIDAR read operations in milliseconds
constexpr int LIDAR_ANGLE_OF_INTEREST_START = 0; // Start angle for LIDAR
constexpr int LIDAR_ANGLE_OF_INTEREST_END = 360; // End angle for LIDAR
constexpr int HEIGHT = 200; // Height of the map in pixels
constexpr int WIDTH = 200; // Width of the map in pixels
#define LIDAR_BAUDRATE 460800 // Common for RPLIDAR S2/A3/M1 (460800). Adjust if using A1/A2 (115200) or other models.
                                // S1/S2/C1 (256000)
#define LIDAR_SERIAL_BUFFER_SIZE 5000 // HardwareSerial TX buffer size for Lidar commands
#define SERVO_DIR_PIN 7     // the servo that modifies the direction of the wheels
#define LIDAR_RX_PIN 5
#define LIDAR_TX_PIN 4
#define LED_PIN LED_BUILTIN

// Harwdare objects
extern Connection connection;
extern HardwareSerial& LIDAR_SERIAL;
extern HardwareSerial ESPS;
extern Lidar lidar;
extern Esp_link esp_link;


// Prototypes of the functions
void setup_test_lidar_basic();              void loop_test_lidar_basic();
void setup_test_connection();               void loop_test_connection();
void setup_test_lidar_express();            void loop_test_lidar_express();
void setup_test_lidar_serial();             void loop_test_lidar_serial();
void setup_test_read_lidar();               void loop_test_read_lidar();
void setup_servo_lidar();                   void loop_servo_lidar();
void setup_test_lidar_express_get_info();   void loop_test_lidar_express_get_info();
void setup_test_lidar_standard();           void loop_test_lidar_standard();
void setup_test_lidar_tcp();                void loop_test_lidar_tcp();
void setup_esps_comm_esp1();                void loop_esps_comm_esp1();
void setup_bayesian_dynamic_tcp();          void loop_bayesian_dynamic_tcp();
void setup_clock_esp1();                    void loop_clock_esp1();
void setup_led_basic();                     void loop_led_basic();
void setup_clock_esp1_AP();                 void loop_clock_esp1_AP();
void setup_send_simple_path();              void loop_send_simple_path();
void setup_send_dynamic_path();             void loop_send_dynamic_path();
