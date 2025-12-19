/**
 * @file config.h
 * @brief Central configuration for ESP2 - all pins, constants, and parameters
 */

#pragma once
#include <cstdint>

namespace Config {
    // ===================================================================
    // HARDWARE PINS
    // ===================================================================
    constexpr uint8_t ESC_PIN = 15;
    constexpr uint8_t SERVO_DIR_PIN = 6;
    constexpr uint8_t US_TRIG_PIN = 5;
    constexpr uint8_t US_ECHO_PIN = 19;
    constexpr uint8_t SDA_PIN = 8;
    constexpr uint8_t SCL_PIN = 9;

    // ===================================================================
    // CONTROL PARAMETERS
    // ===================================================================
    constexpr float MOTOR_POWER_DRIVE = 0.18f;      // 18% throttle
    constexpr float EMERGENCY_DISTANCE = 0.30f;     // meters
    constexpr float GOAL_TOLERANCE = 0.05f;         // 5 cm
    constexpr int STRAIGHT_ANGLE = 90;              //writing 90 degrees into the front wheels servo = straight
    
    // ===================================================================
    // TASK TIMING (milliseconds)
    // ===================================================================
    constexpr uint32_t MOTOR_UPDATE_MS = 20;        // 50 Hz
    constexpr uint32_t ULTRASONIC_UPDATE_MS = 200;  // 5 Hz (slow sensor)
    constexpr uint32_t ODOMETRY_UPDATE_MS = 10;     // 100 Hz
    constexpr uint32_t PURSUIT_UPDATE_MS = 20;      // 50 Hz
    constexpr uint32_t PUBLISH_INTERVAL_MS = 200;   // MQTT rate
    constexpr uint32_t DIAGNOSTICS_INTERVAL_MS = 3000; // 3 seconds

    // ===================================================================
    // WIFI & TCP
    // ===================================================================
    constexpr const char* WIFI_SSID = "LIDAR_AP";
    constexpr const char* WIFI_PASSWORD = "l1darpass";
    constexpr uint16_t TCP_PORT = 9000;
}