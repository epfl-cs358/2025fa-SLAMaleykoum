/**
 * @file global_state.h
 * @brief Global state declarations for ESP2
 * 
 * These variables are defined in src/esp2/global_state.cpp
 * and shared across all tasks.
 */

#pragma once

// Forward declarations (avoid circular includes)
class MotorManager;
class DMS15;
class UltraSonicSensor;
class ImuSensor;
class EncoderCarVelocity;
class Connection;
class PurePursuit;
class Odometry;
class Esp_link;
class WiFiServer;
class WiFiClient;
class GlobalPathMessage;

// ===================================================================
// HARDWARE INSTANCES - defined in global_state.cpp
// ===================================================================
extern MotorManager motor;
extern DMS15 servo_dir;
extern UltraSonicSensor ultrasonic;
extern ImuSensor imu;
extern EncoderCarVelocity encoder;

// ===================================================================
// CONTROL & LOCALIZATION - defined in global_state.cpp
// ===================================================================
extern Connection connection;
extern PurePursuit purePursuit;
extern Odometry odom;
extern Esp_link esp_link;

// ===================================================================
// WIFI & TCP - defined in global_state.cpp
// ===================================================================
extern WiFiServer tcpServer;
extern WiFiClient tcpClient;

// ===================================================================
// STATE FLAGS - defined in global_state.cpp
// ===================================================================
extern bool emergencyStop;
extern bool finishedPath;
extern bool startSignalReceived;
extern bool firstPathReceived;

// ===================================================================
// POSE STATE - defined in global_state.cpp
// ===================================================================
extern float posX;      // meters
extern float posY;      // meters
extern float yaw;       // radians
extern float velocity;  // m/s

// ===================================================================
// PATH DATA - defined in global_state.cpp
// ===================================================================
extern GlobalPathMessage receivedPath;
extern volatile bool newPathArrived;

// ===================================================================
// SHARED BUFFER - defined in global_state.cpp
// ===================================================================
extern char buf[100];