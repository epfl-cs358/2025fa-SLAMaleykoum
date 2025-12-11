/**
 * @file global_state.cpp
 * @brief Global variable definitions for ESP2
 * 
 * These are declared as 'extern' in include/esp2/global_state.h
 * and defined here to avoid multiple definition errors.
 */

#include "esp2/global_state.h"
#include "esp2/config.h"

#include "esp2/hardware/MotorManager.h"
#include "esp2/hardware/DMS15.h"
#include "esp2/hardware/UltraSonicSensor.h"
#include "esp2/hardware/ImuSensor.h"
#include "esp2/hardware/EncoderCarVelocity.h"

#include "esp2/control/pure_pursuit.h"
#include "common/wifi_connection.h"
#include "common/esp_link.h"
#include "common/data_types.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
// ===================================================================
// HARDWARE INSTANCES
// ===================================================================
MotorManager motor(Config::ESC_PIN);
DMS15 servo_dir(Config::SERVO_DIR_PIN);
UltraSonicSensor ultrasonic(Config::US_TRIG_PIN, Config::US_ECHO_PIN);
ImuSensor imu;
EncoderCarVelocity encoder;

// ===================================================================
// CONTROL & LOCALIZATION
// ===================================================================
Connection connection;
PurePursuit purePursuit;
Esp_link esp_link(Serial1);

// ===================================================================
// STATE VARIABLES
// ===================================================================
bool emergencyStop = false;
bool finishedPath = false;
bool startSignalReceived = false;
bool firstPathReceived = false;

bool isCrenFinished = false;
bool isPerformingCreneau = false; 

// Pose state
float posX = 0.0f;
float posY = 0.0f;
float yaw = 0.0f;
float velocity = 0.0f;

Pose2D currentPose = {0.0f, 0.0f, 0.0f, 0};

// Path data
GlobalPathMessage receivedPath = {0};
volatile bool newPathArrived = false;
