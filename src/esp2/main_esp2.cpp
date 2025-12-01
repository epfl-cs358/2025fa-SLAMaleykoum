/**
 * @file main_esp2.cpp
 * @brief Updated Main: Sends dynamic path data over TCP
 */

//Basic
#include <Arduino.h>


//Wifi
#include <WiFi.h>
#include <PubSubClient.h>
#include "common/wifi_connection.h"
#include  "FreeRTOSConfig.h"
#include "common/data_types.h"
#include <cmath>
#include "hardware/I2C_wire.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include "hardware/UltraSonicSensor.h"
#include "hardware/ImuSensor.h"
#include "hardware/I2C_mutex.h"
#include "hardware/EncoderCarVelocity.h"
#include "hardware/MotorController.h"
#include "AS5600.h"
#include "pure_pursuit.h"
#include "common/esp_link.h"
#include "config.h"
#include "global_state.h"
#include "esp2/config.h"
#include "esp2/global_state.h"


TaskHandle_t motorTask, ultrasonicTask, pursuitTask,odomTask,receiveTask;

// ===============================================================
// TASK: Motor Control
// ===============================================================
void TaskMotor(void *pvParameters) {
    for (;;) {
        if (!startSignalReceived) {
            motor.stop();
            servo_dir.setAngle(90);
        } 
        else if (emergencyStop || finishedPath) {
            motor.stop();
            servo_dir.setAngle(90);
        } else {
            motor.forward(Config::MOTOR_POWER_DRIVE);
        }
        motor.update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ===============================================================
// TASK: Ultrasonic Emergency Stop
// ===============================================================
void TaskUltrasonic(void *pvParameters) {
    for (;;) {
        if (!startSignalReceived) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        float dist = ultrasonic.readDistance();
        if (dist > 0 && dist < Config::EMERGENCY_DISTANCE) {
            motor.stop();
            motor.update();
            emergencyStop = true;
            
        } 
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ===============================================================
// TASK: IMU Orientation
// ===============================================================
float getYawIMU(const IMUData& imu_data) {
  float qw = imu_data.qw;
  float qx = imu_data.qx;
  float qy = imu_data.qy;
  float qz = imu_data.qz;
  float siny_cosp = 2.0 * (qw * qz + qx * qy);
  float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return atan2(siny_cosp, cosy_cosp);
}

void TaskOdometryUnified_Stable(void *pvParameters) {
    uint32_t prevTime = micros();
    uint32_t lastPrintTime = 0;
    for (;;) {
        uint32_t now = micros();
        float dt = (now - prevTime) / 1000000.0f;
        prevTime = now;
        
        i2c_lock();
        imu.readAndUpdate();
        IMUData imuData = imu.data();
        float currentYaw = getYawIMU(imuData);
        float currentVel = encoder.getWheelLinearVelocity();
        i2c_unlock();
        
        yaw = currentYaw;
        velocity = currentVel;
        
        posX -= velocity * sinf(yaw) * dt;
        posY += velocity * cosf(yaw) * dt;


        //Send pose to the esp1 after each computation of the position
        Pose2D pose;
        pose.x = posX;
        pose.y = posY;
        pose.theta = yaw;
        pose.timestamp_ms = millis();
        
        esp_link.sendPos(pose);
        
        //print -? mqtt or other method?
        if (millis() - lastPrintTime > 1000) {
            Serial.printf("Pose TX: X=%.3f Y=%.3f θ=%.3f | 100Hz\n", posX, posY, yaw);
            lastPrintTime = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===============================================================
// TASK: Pure Pursuit Controller
// ===============================================================
void TaskPurePursuit(void *pvParameters) {
    
    // Initial wait for at least one path
    while (!newPathArrived) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (;;) {
        // --- 1. Path Update Logic ---
        if (newPathArrived) {
            // Update the controller
            purePursuit.set_path(receivedPath);
            finishedPath = false; // Reset finish state for new path
            
            // Send Path to Python Logger
            if (tcpClient && tcpClient.connected()) {
                // Send Header
                tcpClient.printf("PATH_START: ID=%d LEN=%d\n", receivedPath.path_id, receivedPath.current_length);
                // Send Waypoints
                for(int i=0; i<receivedPath.current_length; i++) {
                    tcpClient.printf("WP: %d %.3f %.3f\n", i, receivedPath.path[i].x, receivedPath.path[i].y);
                    vTaskDelay(2); // Small delay to prevent TCP buffer overflow
                }
                tcpClient.println("PATH_END");
            }
            newPathArrived = false;
        }

        // --- 2. TCP Connection ---
        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                Serial.printf("Client connected from %s\n", tcpClient.remoteIP().toString().c_str());
            }
        }

        // --- 3. Read Commands ---
        if (tcpClient && tcpClient.connected()) {
            while (tcpClient.available()) {
                String line = tcpClient.readStringUntil('\n');
                line.trim();
                if (line == "start") {
                    startSignalReceived = true;
                    Serial.println("✅ Start command received!");
                }
            }
        }

        if (!startSignalReceived) {
            motor.stop();
            motor.update();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        Pose2D currentPose = {posX, posY, yaw};
        Velocity velStruct = {velocity, 0.0f};

        MotionCommand cmd = purePursuit.compute_command(currentPose, velStruct);
        
        Waypoint lh = purePursuit.get_lookahead_point();
        float Kp_val = purePursuit.get_kp();
        float Ld_val = purePursuit.get_ld();

        // --- 4. Send Telemetry ---
        if (tcpClient && tcpClient.connected()) {
            char tcpBuf[256];
            int len = snprintf(tcpBuf, sizeof(tcpBuf),
                               "CMD: v=%.3f d=%.3f | P: Kp=%.2f Ld=%.2f | T: LX=%.3f LY=%.3f | POSE: X=%.2f Y=%.2f Yaw=%.2f Vel=%.3f\n",
                               cmd.v_target, cmd.delta_target,
                               Kp_val, Ld_val,
                               lh.x, lh.y,
                               posX, posY, yaw, velocity);
            tcpClient.write((const uint8_t*)tcpBuf, len);
        }

        servo_dir.setAngle(cmd.delta_target);

        if ((cmd.v_target == 0)){
            motor.stop();
            motor.update();
            finishedPath = true;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void TaskReceivePath(void *pvParameters) {
    for (;;) {
        esp_link.poll();
        GlobalPathMessage gpm;
        if (esp_link.get_path(gpm)) {
            receivedPath = gpm;
            // Set flag so Pure Pursuit task picks it up and sends it to TCP
            newPathArrived = true; 
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===============================================================
// SETUP
// ===============================================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.printf("Starting WiFi AP '%s' ...\n", Config::WIFI_SSID);
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(Config::WIFI_SSID, Config::WIFI_PASSWORD);
    if (!ok) Serial.println("Failed to start AP!");
    else Serial.println(WiFi.softAPIP());

    tcpServer.begin();
    tcpServer.setNoDelay(true);

    I2C_wire.begin(Config::SDA_PIN,Config::SCL_PIN);
    i2cMutexInit();
    esp_link.begin();

    if (!servo_dir.begin()) { Serial.println("Servo init failed!"); while (true); }
    servo_dir.setAngle(90);

    if (!ultrasonic.begin()) { Serial.println("WARNING: UltraSonic init failed. Continuing..."); } 
    else { Serial.println("✅ Ultra Sonic initialized"); }

    if (!imu.begin()) { Serial.println("IMU init failed!"); vTaskDelete(NULL); return; }
    if (!encoder.begin()) { Serial.println("Encoder init failed!"); while (true); }
    if (!motor.begin()) { Serial.println("Motor init failed!"); while (true); }

    xTaskCreatePinnedToCore(TaskReceivePath,"RX_Path",4096,NULL,3,&receiveTask,0);
    xTaskCreatePinnedToCore(TaskOdometryUnified_Stable, "OdomStable", 8192, NULL, 3, &odomTask, 0);
    xTaskCreatePinnedToCore(TaskMotor,       "Motor",       4096, NULL, 2, &motorTask,   1); 
    xTaskCreatePinnedToCore(TaskUltrasonic,  "Ultrasonic",  4096, NULL, 2, &ultrasonicTask,1); 
    xTaskCreatePinnedToCore(TaskPurePursuit, "PurePursuit", 4096, NULL, 1, &pursuitTask, 1); 
}

void loop() { }
