/**
 * @file main_esp2.cpp
 * @brief Updated Main with Lookahead and Parameter Transmission
 */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "common/wifi_connection.h"
#include  "FreeRTOSConfig.h"
#include "common/data_types.h"
#include <cmath>
#include "I2C_wire.h"
#include "MotorManager.h"
#include "DMS15.h"
#include "UltraSonicSensor.h"
#include "ImuSensor.h"
#include "I2C_mutex.h"
#include "EncoderCarVelocity.h"
#include "MotorController.h"
#include "AS5600.h"
#include "motor_pid.h"
#include "pure_pursuit.h"
#include "odometry.h"


#include "common/esp_link.h"

Esp_link esp_link(Serial1);

// Storage for last received global path
GlobalPathMessage receivedPath;
bool pathAvailable = false;

// ---- ALEX TCP STUFFFF ----
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

const char* ssid = "LIDAR_AP";
const char* password = "l1darpass";
const uint16_t TCP_PORT = 9000;

WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
// ---- END ALEX TCP STUFFFF ----

#define ESC_PIN 15          
#define SERVO_DIR_PIN 6     
#define US_TRIG_PIN 5       
#define US_ECHO_PIN 19      
#define  SDA_PIN 8           
#define SCL_PIN 9           

MotorManager motor(ESC_PIN);
DMS15 servo_dir(SERVO_DIR_PIN);
UltraSonicSensor ultrasonic(5,19);
ImuSensor imu;
EncoderCarVelocity encoder;
PurePursuit purePursuit;
Odometry odom;

bool emergencyStop = false;
bool finishedPath = false;
bool startSignalReceived = false; 
float motorPowerDrive = 0.20f;
char buf[256]; // Increased buffer size

// --- Pose estimation ---
float velocity = 0.0f;
float posX = 0.0f;
float posY = 0.0f;
float yaw = 0.0f;
float newY = 0.0f;

// --- Path definition ---
//0.40m y, 3.40m x, 1.80m y, -3.6m x

float pathX[] = {
    0.00, 0.00, 0.50, 1.00, 1.50, 2.00, 2.50, 3.00, 3.50, 4.00, 4.00, 4.00, 4.00, 4.00, 4.00, 3.50, 3.00, 2.50, 2.00, 1.50, 1.00, 0.50, 0.00, -0.20
};
float pathY[] = {
    0.00, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.50, 1.00, 1.50, 2.00, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40
};
const int pathSize = sizeof(pathX) / sizeof(pathX[0]);

const float EMERGENCY_DISTANCE = 0.25f;
TaskHandle_t motorTask, ultrasonicTask, imuTask, poseTask, pursuitTask;

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
            motor.forward(motorPowerDrive);
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
        if (dist > 0 && dist < EMERGENCY_DISTANCE) {
            motor.stop();
            motor.update();
            emergencyStop = true;
            snprintf(buf, sizeof(buf), "⚠️ Emergency stop! Distance=%.2f m", dist);
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
    
    for (;;) {
        uint32_t now = micros();
        float dt = (now - prevTime) / 1000000.0f;
        prevTime = now;
        
        i2c_lock();
        imu.readAndUpdate();
        IMUData imuData = imu.data();
        float currentYaw = getYawIMU(imuData);
        float currentVel = encoder.getWheelLinearVelocity();
        float currentDist = encoder.getDistance();
        i2c_unlock();
        
        yaw = currentYaw;
        velocity = currentVel;
        newY = currentDist;
        
        posX -= velocity * sinf(yaw) * dt;
        posY += velocity * cosf(yaw) * dt;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===============================================================
// TASK: Pure Pursuit Controller
// ===============================================================
void TaskPurePursuit(void *pvParameters) {
    // Wait until ESP1 sends the path
    while (!pathAvailable) {
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    purePursuit.set_path(receivedPath);

    

    for (;;) {
        // 1. Accept new clients
        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                Serial.printf("Client connected from %s\n", tcpClient.remoteIP().toString().c_str());
            }
        }

        // 2. Read incoming commands
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
        
        // Retrieve internal state from PurePursuit
        Waypoint lh = purePursuit.get_lookahead_point();
        float Kp_val = purePursuit.get_kp();
        float Ld_val = purePursuit.get_ld();

        // 3. Send Telemetry
        if (tcpClient && tcpClient.connected()) {
            char tcpBuf[256];
            // FORMAT: CMD | PARAMS | TARGET | POSE
            int len = snprintf(tcpBuf, sizeof(tcpBuf),
                               "CMD: v=%.3f d=%.3f | P: Kp=%.2f Ld=%.2f | T: LX=%.3f LY=%.3f | POSE: X=%.2f Y=%.2f Yaw=%.2f Vel=%.3f\n",
                               cmd.v_target, cmd.delta_target,
                               Kp_val, Ld_val,
                               lh.x, lh.y,
                               posX, posY, yaw, velocity);
            tcpClient.write((const uint8_t*)tcpBuf, len);
        }

        servo_dir.setAngle(cmd.delta_target);
        // motorPowerDrive = speed_to_power_drive(cmd.v_target);

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

            // Store the received path
            receivedPath = gpm;

            purePursuit.set_path(receivedPath);

            pathAvailable = true;



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

    Serial.printf("Starting WiFi AP '%s' ...\n", ssid);
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(ssid, password);
    if (!ok) Serial.println("Failed to start AP!");
    else Serial.println(WiFi.softAPIP());

    tcpServer.begin();
    tcpServer.setNoDelay(true);

    I2C_wire.begin(8, 9);
    i2cMutexInit();

    esp_link.begin();

    if (!servo_dir.begin()) { Serial.println("Servo init failed!"); while (true); }
    servo_dir.setAngle(90);

    if (!ultrasonic.begin()) {
        Serial.println("WARNING: UltraSonic init failed. Continuing...");
    } else {
        Serial.println("✅ Ultra Sonic initialized");
    }

    if (!imu.begin()) { Serial.println("IMU init failed!"); vTaskDelete(NULL); return; }
    if (!encoder.begin()) { Serial.println("Encoder init failed!"); while (true); }
    if (!motor.begin()) { Serial.println("Motor init failed!"); while (true); }

    xTaskCreatePinnedToCore(TaskReceivePath,"RX_Path",4096,NULL,3,NULL,0);
    xTaskCreatePinnedToCore(TaskOdometryUnified_Stable, "OdomStable", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TaskMotor,       "Motor",       4096, NULL, 2, &motorTask,   1); 
    xTaskCreatePinnedToCore(TaskUltrasonic,  "Ultrasonic",  4096, NULL, 2, &ultrasonicTask,1); 
    xTaskCreatePinnedToCore(TaskPurePursuit, "PurePursuit", 4096, NULL, 1, &pursuitTask, 1); 
}

void loop() { 
}