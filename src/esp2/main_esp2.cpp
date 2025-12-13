#include <Arduino.h>
#include <WiFi.h>

// 1. Configuration & Global State
#include "config.h"
#include "global_state.h"

// 2. Hardware Interfaces (Required for .begin() calls)
#include "hardware/I2C_wire.h"
#include "hardware/I2C_mutex.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include "hardware/UltraSonicSensor.h"
#include "hardware/ImuSensor.h"
#include "hardware/EncoderCarVelocity.h"
#include "common/esp_link.h"

// 3. Task Definitions (Required for xTaskCreate)
#include "tasks/task_motor.h"
#include "tasks/task_ultrasonic.h"
#include "tasks/task_odometry.h"
#include "tasks/task_pure_pursuit.h"
#include "tasks/task_receive_path.h"

// Task Handles
TaskHandle_t motorTask, ultrasonicTask, pursuitTask, odomTask, receiveTask;

void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Turn off built-in LED

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

    // delay(500);
    // digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED

    // --- FreeRTOS Tasks Creation ---
    xTaskCreatePinnedToCore(TaskReceivePath,            "RX_Path",      4096, NULL, 3, &receiveTask,    0);
    xTaskCreatePinnedToCore(TaskOdometryUnified_Stable, "Odom",         8192, NULL, 3, &odomTask,       0);
    xTaskCreatePinnedToCore(TaskMotor,                  "Motor",        4096, NULL, 2, &motorTask,      1); 
    xTaskCreatePinnedToCore(TaskUltrasonic,             "Ultrasonic",   4096, NULL, 2, &ultrasonicTask, 1); 
    xTaskCreatePinnedToCore(TaskPurePursuit,            "PurePursuit",  4096, NULL, 1, &pursuitTask,    1); 


    digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH); // Turn off built-in LED
    delay(200);
    digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH); // Turn off built-in LED
    delay(200);
    digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED
}

void loop() { 
    // Main loop is empty because all operations are handled in FreeRTOS tasks
    // Once the taks are created in setup(), they run independently
}
