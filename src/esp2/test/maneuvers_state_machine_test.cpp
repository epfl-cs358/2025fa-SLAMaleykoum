#include <Arduino.h>
#include <WiFi.h>

// 1. Configuration & Global State
#include "config.h"
#include "global_state.h"

// 2. Hardware Interfaces
#include "hardware/I2C_wire.h"
#include "hardware/I2C_mutex.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include "hardware/UltraSonicSensor.h"
#include "hardware/ImuSensor.h"
#include "hardware/EncoderCarVelocity.h"
#include "common/esp_link.h"

// 3. Task Definitions
#include "tasks/task_motor.h"
#include "tasks/task_ultrasonic.h"
#include "tasks/task_odometry.h"
#include "tasks/task_pure_pursuit.h"
#include "tasks/task_receive_path.h"

// ============================================
// TEST MODE CONFIGURATION
// ============================================
#define TEST_MODE_ENABLED true  // Set to false for normal operation

// Task Handles
TaskHandle_t motorTaskManeuvers, ultrasonicTaskManeuvers, pursuitTaskManeuvers, odomTaskManeuvers, receiveTaskManeuvers, testTaskManeuvers;

// ============================================
// TEST TASK - Simulates ESP1 Behavior
// ============================================
void TaskManeuversTestSimulator(void *pvParameters) {
    Serial.println("=== TEST MODE ACTIVE ===");
    Serial.println("Waiting 3 seconds before starting...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Step 1: Send initial "fake" path to start the car
    Serial.println("[TEST] Sending initial path to start car...");
    receivedPath.current_length = 3;
    receivedPath.path_id = 1;
    receivedPath.timestamp_ms = millis();
    
    // Create a straight path ahead (just to get the car moving)
    receivedPath.path[0] = {0.0f, 1.0f};  // 1m ahead
    receivedPath.path[1] = {0.0f, 2.0f};  // 2m ahead
    receivedPath.path[2] = {0.0f, 3.0f};  // 3m ahead
    
    firstPathReceived = true;
    newPathArrived = true;
    
    Serial.println("[TEST] Car should start driving forward...");
    Serial.println("[TEST] Put your hand in front of ultrasonic to trigger emergency stop!");
    
    // Step 2: Wait for emergency stop to be triggered
    while (!emergencyStop) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    Serial.println("[TEST] Emergency stop detected!");
    Serial.println("[TEST] Waiting 2 seconds before sending recovery path...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Step 3: Send recovery path with first waypoint at 45 degrees
    Serial.println("[TEST] Sending recovery path (45° angle)...");
    
    // Calculate a point 1 meter away at 45 degrees from current position
    float currentX = currentPose.x;
    float currentY = currentPose.y;
    float currentYawDeg = currentPose.theta * 180.0 / PI;
    
    // Calculate 45° relative to current heading
    float targetAngleRad = (currentYawDeg + 45.0) * PI / 180.0;
    float targetX = -0.50f;//currentX + 1.0 * cos(targetAngleRad);
    float targetY = 0.0f;//currentY + 1.0 * sin(targetAngleRad);
    
    receivedPath.current_length = 3;
    receivedPath.path_id = 2;  // New path ID
    receivedPath.timestamp_ms = millis();
    
    receivedPath.path[0] = {-0.50f, 0.0f};  // 45° turn target
    receivedPath.path[1] = {0.0f, 0.0f};
    receivedPath.path[2] = {0.0f,0.0f};
    
    newPathArrived = true;
    
    Serial.print("[TEST] Recovery path sent. First waypoint: (");
    Serial.print(targetX);
    Serial.print(", ");
    Serial.print(targetY);
    Serial.println(")");
    Serial.println("[TEST] Car should now turn and resume driving...");
    
    // Step 4: Monitor the recovery
    vTaskDelay(pdMS_TO_TICKS(2000));
    Serial.println("[TEST] Monitoring recovery...");
    
    for (;;) {
        Serial.print("[TEST] Emergency Stop: ");
        Serial.print(emergencyStop ? "YES" : "NO");
        Serial.print(" | Current Yaw: ");
        Serial.print(currentPose.theta * 180.0 / PI);
        Serial.print("° | Position: (");
        Serial.print(currentPose.x);
        Serial.print(", ");
        Serial.print(currentPose.y);
        Serial.println(")");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================
// SETUP
// ============================================
void setup_maneuvers_state_machine_test() {
    Serial.begin(115200);
    delay(2000);

    // --- Hardware Init ---
    I2C_wire.begin(Config::SDA_PIN, Config::SCL_PIN);
    i2cMutexInit();
    esp_link.begin();

    if (!servo_dir.begin()) { Serial.println("Servo init failed!"); while (true); }
    else { Serial.println("Servo initialized"); }
    servo_dir.setAngle(90);
    
    if (!ultrasonic.begin()) { Serial.println("WARNING: US init failed"); } 
    else { Serial.println("Ultra Sonic initialized"); }
    
    if (!imu.begin()) { Serial.println("IMU init failed!"); vTaskDelete(NULL); return; }
    else { Serial.println("IMU initialized"); }
    
    if (!encoder.begin()) { Serial.println("Encoder init failed!"); while (true); }
    else { Serial.println("Encoder initialized"); }
    
    if (!motor.begin()) { Serial.println("Motor init failed!"); while (true); }
    else { Serial.println("Motor initialized"); }

    Serial.println("---- Initialization complete. ----");

    // --- FreeRTOS Tasks Creation ---
    if (TEST_MODE_ENABLED) {
        Serial.println("=== STARTING IN TEST MODE ===");
        // Don't create the receive path task (we'll simulate it)
        xTaskCreatePinnedToCore(TaskManeuversTestSimulator,             "TestSim",      4096, NULL, 3, &testTaskManeuvers,       0);
    } else {
        xTaskCreatePinnedToCore(TaskReceivePath,               "RX_Path",      4096, NULL, 3, &receiveTaskManeuvers,    0);
    }
    
    xTaskCreatePinnedToCore(TaskOdometryUnified_Stable, "Odom",         8192, NULL, 3, &odomTaskManeuvers,       0);
    xTaskCreatePinnedToCore(TaskMotor,                  "Motor",        4096, NULL, 2, &motorTaskManeuvers,      1); 
    xTaskCreatePinnedToCore(TaskUltrasonic,             "Ultrasonic",   4096, NULL, 2, &ultrasonicTaskManeuvers, 1); 
    xTaskCreatePinnedToCore(TaskPurePursuit,            "PurePursuit",  4096, NULL, 1, &pursuitTaskManeuvers,    1); 
}

void loop_maneuvers_state_machine_test() { 
    // Empty - everything runs in tasks
}