/**
 * @file main_esp2.cpp
 * @brief Main control program for the SLAMaleykoum autonomous car.
 *
 * This file contains the primary firmware logic executed on the ESP32-S3 board.  
 * It integrates all hardware modules — motor, servo, ultrasonic sensor, IMU, and encoder — 
 * as well as the Wi-Fi and MQTT connection system.  
 * The current implementation serves as a comprehensive integration test, 
 * which will later evolve into the final autonomous driving program.
 *
 * @note
 * This version is primarily designed for validation and debugging.  
 * The final release will include autonomous navigation logic, data fusion, 
 * and closed-loop control algorithms.
 *
 * @author SLAMaleykoum
 * @date October 2025
 */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "common/wifi_connection.h"

#include  "FreeRTOSConfig.h"
#include "common/data_types.h"
#include <Arduino.h>
#include <cmath>
#include "I2C_wire.h"

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
#include "pure_pursuit.h"
#include "odometry.h"

#define ESC_PIN 15          // pin used for the motor
#define SERVO_DIR_PIN 6     // the servo that modifies the direction of the wheels
#define US_TRIG_PIN 5       // changé par rapport a avant sur conseil de chat
#define US_ECHO_PIN 19      // changé par rapport a avant sur conseil de chat
#define  SDA_PIN 8           // pin used for SDA of IMU
#define SCL_PIN 9           // pin used for SCL of IMU


// ---- ALEX TCP STUFFFF ----
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

const char* ssid = "LIDAR_AP";
const char* password = "l1darpass";
const uint16_t TCP_PORT = 9000;

// TCP server and client
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
// ---- END ALEX TCP STUFFFF ----


MotorManager motor(ESC_PIN);
//MotorController motor(ESC_PIN);
DMS15 servo_dir(SERVO_DIR_PIN);
UltraSonicSensor ultrasonic(5,19);//US_TRIG_PIN, US_ECHO_PIN);
ImuSensor imu(SDA_PIN, SCL_PIN);
//AS5600Encoder encoder(SDA_PIN, SCL_PIN);
EncoderCarVelocity encoder;
// Connection connection;
MotorPID pid(0.9f, 0.1f, 0.0f);
PurePursuit purePursuit;//(0.26f);
Odometry odom;


#include "common/data_types.h"
#include <Arduino.h>
#include <cmath>
#include "I2C_wire.h"


// const char* mqtt_topic_connection_pure_pursuit = "slamaleykoum77/print";
// const char* mqtt_topic_connection_location = "slamaleykoum77/purepursuit";


// === Global instances ===


bool emergencyStop = false;
bool finishedPath = false;
bool startSignalReceived = false; 
float motorPowerDrive = 0.18f;  // 18%


char buf[100];


// --- Pose estimation ---

float velocity = 0.0f;
float posX = 0.0f;
float posY = 0.0f;
float yaw = 0.0f;
float newY = 0.0f;

// --- Path definition ---

// float pathX[] = {0, 1};
// float pathY[] = {0, 1};

// Sample path: a straight line for 1m from (0, 0) to (0, 1) followed by a sharp right
// turn and another 1m straight line from (0, 1) to (1, 1) with waypoints every 10cm.
// Sample path: 
float pathX[] = {
    0.00, 0.00, 0.00, 0.00, 0.50, 1.50, 2.00, 2.50, 2.80, 2.80, 2.80, 2.80, 2.80
};
float pathY[] = {
    0.00, 0.50, 1.00, 1.50, 1.50, 1.50, 1.50, 1.50, 1.50, 2.00, 2.50, 2.00, 3.40
};


const int pathSize = sizeof(pathX) / sizeof(pathX[0]);

// --- Constants ---
const float EMERGENCY_DISTANCE = 0.25f;
const float GOAL_TOLERANCE = 0.05f;  // 5 cm

// --- FreeRTOS handles ---
TaskHandle_t motorTask, ultrasonicTask, imuTask, poseTask, pursuitTask;

// ===============================================================
// TASK: Motor Control
// ===============================================================
void TaskMotor(void *pvParameters) {
    
    for (;;) {
        // If we haven't started, keep motor stopped and servo straight
        if (!startSignalReceived) {
            motor.stop();
            servo_dir.setAngle(90);
        } 
        else if (emergencyStop || finishedPath) {
            motor.stop();
            servo_dir.setAngle(90);
        } else {
            motor.forward(motorPowerDrive);  // already normalized [0,1]
            
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
        // --- NEW: Wait for start signal ---
        // We do NOT want to read distance or trigger emergency stop
        // before the computer says "start".
        if (!startSignalReceived) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Sleep 100ms and check again
            continue;
        }

        float dist = ultrasonic.readDistance();
        if (dist > 0 && dist < EMERGENCY_DISTANCE) {
            motor.stop();
            motor.update();

            emergencyStop = true;
            
            snprintf(buf, sizeof(buf), "⚠️ Emergency stop! Distance=%.2f m", dist);
            // connection.publish(mqtt_topic_connection_location, buf);

             

        } /*else if (dist > (EMERGENCY_DISTANCE + 0.05f)) {
            emergencyStop = false; //this will make it restart so if you want it to stay stopped keeep commented out
        }*/
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ===============================================================
// TASK: IMU Orientation (yaw extraction)
// ===============================================================

float getYawIMU(const IMUData& imu_data) {
  // Extract quaternion components
  float qw = imu_data.qw;
  float qx = imu_data.qx;
  float qy = imu_data.qy;
  float qz = imu_data.qz;

  // Yaw (z-axis rotation)
  // This is the standard formula for quaternion-to-euler conversion
  float siny_cosp = 2.0 * (qw * qz + qx * qy);
  float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  float yaw = atan2(siny_cosp, cosy_cosp);

  return yaw; // Yaw angle in radians
}



void TaskOdometryUnified_Stable(void *pvParameters) {
    uint32_t prevTime = micros();
    static uint32_t lastPublishTime = 0;
    
    for (;;) {
        uint32_t now = micros();
        float dt = (now - prevTime) / 1000000.0f;
        prevTime = now;
        
        // Read sensors
        i2c_lock();
        imu.readAndUpdate();
        IMUData imuData = imu.data();
        float currentYaw = getYawIMU(imuData);
        
        float currentVel = encoder.getWheelLinearVelocity();
        float currentDist = encoder.getDistance();
        i2c_unlock();
        
        // Update globals
        yaw = currentYaw;
        velocity = currentVel;
        newY = currentDist;
        
        // Update position
        posX -= velocity * sinf(yaw) * dt;
        posY += velocity * cosf(yaw) * dt;
        
        // Publish
        uint32_t nowMillis = millis();
        if (nowMillis - lastPublishTime >= 200) {
            // connection.check_connection();
            // snprintf(buf, sizeof(buf),
            //          "Pose: X=%.3f Y=%.3f Yaw=%.1f° V=%.3f dt=%.4f",
            //          posX, posY, yaw * 180.0f / M_PI, velocity, dt);
            // connection.publish(mqtt_topic_connection_location, buf);
            lastPublishTime = nowMillis;
        }
        
        // Run at 100 Hz (10ms interval) - more stable
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}






void TaskIMU(void *pvParameters) {
   
    for (;;) {
        i2c_lock();
        imu.readAndUpdate();
        IMUData q = imu.data();
        i2c_unlock();

        // ✅ PRINT QUATERNIONS FIRST TO SEE IF THEY CHANGE
        snprintf(buf, sizeof(buf), "RAW: qw=%.4f qx=%.4f qy=%.4f qz=%.4f", 
                 q.qw, q.qx, q.qy, q.qz);
        //connection.publish(mqtt_topic_connection_pure_pursuit, buf);
        
        // ✅ COMPUTE YAW
        float yaw_radians = getYawIMU(q);//odom.computeYaw(q);
        yaw = yaw_radians;


        vTaskDelay(pdMS_TO_TICKS(5));  // Slow down for debugging
    }
}






//Task for encodervelocity
void TaskEncoder(void *pvParameters) {
    
    for (;;) {
        i2c_lock();
        float currVel = encoder.getWheelLinearVelocity();   // updates internal velocity state
        newY = encoder.getDistance();
        i2c_unlock();

        velocity = currVel;
        //connection.check_connection();
        snprintf(buf, sizeof(buf), "Current Vel=%.3f",
                     currVel);
        //connection.publish(mqtt_topic_connection_pure_pursuit, buf);

        vTaskDelay(pdMS_TO_TICKS(5));       // 200 Hz
    }
}/*

void TaskOdometry(void *pvParameters)
{
    for(;;)
    {
        i2c_lock();
        imu.readAndUpdate();
        IMUData imuData = imu.data();
        i2c_unlock();

        i2c_lock();
        float currVel = encoder.getWheelLinearVelocity();   // updates internal velocity state
        newY = encoder.getDistance();
        i2c_unlock();

        velocity = currVel;

        odom.update(imuData, currVel);

        vTaskDelay(pdMS_TO_TICKS(20));       // 50 Hz
    }
}*/

// ===============================================================
// TASK: Pose Estimation (dead reckoning)
// ===============================================================
void TaskPose(void *pvParameters) {
    float prevTime = millis();
    static uint32_t lastPublishTime = 0;

    for (;;) {
        float now = millis();
        float dt = (now - prevTime) / 1000.0f;
        //float now = micros() / 1e6;
        //float dt = now - prevTime;
        prevTime = now;

        // yaw and velocity are updated by TaskIMU and TaskEncoder

        posX -= velocity * sinf(yaw) * dt;
        posY += velocity * cosf(yaw) * dt;
        /*
        snprintf(buf, sizeof(buf),
                     "Pose: X=%.2f Y=%.2f  newY=%.2f Yaw=%.2f Vel=%.3f",
                     posX, posY, newY, yaw, velocity);
        connection.publish(mqtt_topic_connection_pure_pursuit, buf);*/

        // if (now - lastPublishTime >= 200) {
        //     connection.check_connection();
        //     snprintf(buf, sizeof(buf),
        //              "Pose: X=%.2f Y=%.2f  newY=%.2f Yaw=%.2f Vel=%.3f",
        //              posX, posY, newY, yaw, velocity);
        //     connection.publish(mqtt_topic_connection_location, buf);
        //     lastPublishTime = now;
        // }

        vTaskDelay(pdMS_TO_TICKS(5));  // 50 Hz
    }
}

// ===============================================================
// TASK: Pure Pursuit Controller
// ===============================================================
void TaskPurePursuit(void *pvParameters) {
    // Prepare path
    GlobalPathMessage msg;
    msg.current_length = pathSize;
    msg.timestamp_ms = millis();
    msg.path_id = 1;
    for (int i = 0; i < pathSize; i++) {
        msg.path[i].x = pathX[i];
        msg.path[i].y = pathY[i];
    }

    purePursuit.set_path(msg);

    for (;;) {
        // ---- ALEX TCP RECEIVE STUFF ----
        // 1. Accept new clients if needed
        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                Serial.printf("Client connected from %s\n",
                                tcpClient.remoteIP().toString().c_str());
            }
        }

        // 2. Read incoming commands (Look for "start")
        if (tcpClient && tcpClient.connected()) {
            while (tcpClient.available()) {
                String line = tcpClient.readStringUntil('\n');
                line.trim();
                if (line == "start") {
                    startSignalReceived = true;
                    Serial.println("✅ Start command received from Logger!");
                }
            }
        }
        // ---- END ALEX TCP RECEIVE STUFF ----

        // If we haven't received the start command, wait.
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
        //Pose2D currentPose = { odom.x(), odom.y(), odom.yaw() };
        Velocity velStruct = {velocity, 0.0f};

        MotionCommand cmd = purePursuit.compute_command(currentPose, velStruct);

        // ---- ALEX TCP SEND STUFF ----
        if (tcpClient && tcpClient.connected()) {
            char tcpBuf[100];
            // Send command and current pose information over TCP
            int len = snprintf(tcpBuf, sizeof(tcpBuf),
                               "CMD: v_target=%.3f delta_target=%.3f | POSE: X=%.2f Y=%.2f Yaw=%.2f Vel=%.3f\n",
                               cmd.v_target, cmd.delta_target,
                               posX, posY, yaw, velocity);
            tcpClient.write((const uint8_t*)tcpBuf, len);
        }
        // ---- END ALEX TCP SEND STUFF ----

        // Steering 
        //float steeringDeg = cmd.delta_target * 180.0f / M_PI;
        servo_dir.setAngle(cmd.delta_target);

        snprintf(buf, sizeof(buf),
                "Pose: X=%.2f Y=%.2f Yaw=%.2f Vel=%.3f newY=%.2f",
                posX, posY, yaw, velocity, newY);
            //connection.publish(mqtt_topic_connection_location, buf);


        if ((cmd.v_target == 0)){//(distToGoal < GOAL_TOLERANCE)) {
            motor.stop();
            motor.update();
            motor.stop();
            motor.update();
            motor.stop();
            motor.update();
            
            // connection.check_connection();

            // snprintf(buf, sizeof(buf), "Purepursuit says v = 0 ");
            // connection.publish(mqtt_topic_connection_location, buf);
            finishedPath = true;
            // delay(6000);
            // snprintf(buf, sizeof(buf), "✅ Reached goal at (%.2f, %.2f, %.2f), stopping.", posX, posY,newY);
            // connection.publish(mqtt_topic_connection_location, buf);



            finishedPath = true;
            
            motor.stop();
            motor.update();
            
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ===============================================================
// SETUP & LOOP
// ===============================================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    // ---- ALEX TCP STUFFFF ----
    Serial.printf("Starting WiFi AP '%s' ...\n", ssid);
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(ssid, password);
    if (!ok) {
        Serial.println("Failed to start AP!");
    } else {
        IPAddress ip = WiFi.softAPIP();
        Serial.print("AP started. IP: ");
        Serial.println(ip);
        Serial.printf("Connect your PC to %s (password: %s)\n", ssid, password);
    }

    // Start TCP server
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("TCP server started on port %u\n", TCP_PORT);
    // ---- END ALEX TCP STUFFFF ----


    // connection.setupWifi();

    I2C_wire.begin(8, 9);
    // I2C setup for IMU and Encoder
    i2cMutexInit();

    // Wifi connection 
    // connection.check_connection();


    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }
    servo_dir.setAngle(90);

    
    if (!ultrasonic.begin()) {
        char msgSonic[50];
        // FIX: Replaced "while(true)" with a simple print. 
        // If the sensor fails (no echo), we continue anyway so TCP can start.
        snprintf(msgSonic, sizeof(msgSonic), "WARNING: UltraSonic init failed (check wiring or obstacle distance). Continuing...");
        Serial.println(msgSonic);
        // while (true); <--- THIS WAS THE BLOCKER
    } else {
        snprintf(buf, sizeof(buf), "✅ Ultra Sonic initialized successfully");
        Serial.println(buf);
    }
    



    if (!imu.begin()) {
        snprintf(buf, sizeof(buf), "❌ IMU failed to initialize!");
        // connection.publish(mqtt_topic_connection_pure_pursuit, buf);
        vTaskDelete(NULL);
        return;
    }
   

    snprintf(buf, sizeof(buf), "✅ IMU initialized successfully");
    // connection.publish(mqtt_topic_connection_pure_pursuit, buf);
    

    if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        // connection.publish(mqtt_topic_connection_pure_pursuit,msgEncoder);
        while (true);
    }
    

    char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder setup successful!");
    // connection.publish(mqtt_topic_connection_pure_pursuit,msgEncoder);

    

    if (!motor.begin()) {
        char msgMotor[50];
        snprintf(msgMotor, sizeof(msgMotor), "Motor init failed!");
        // connection.publish(mqtt_topic_connection_pure_pursuit , msgMotor);
        while (true);
    }

    snprintf(buf, sizeof(buf), "✅ Motor initialized successfully");
    // connection.publish(mqtt_topic_connection_pure_pursuit, buf);



    snprintf(buf, sizeof(buf), "🚗 FreeRTOS Path-Follow Test Start");
    //connection.publish(mqtt_topic_connection_pure_pursuit, buf);


    
    //Core 0 from high to low priority : I2C
    //xTaskCreatePinnedToCore(TaskIMU,          "IMU",          4096, NULL, 3,  &imuTask,        0); //core 0
    //xTaskCreatePinnedToCore(TaskEncoder,      "Encoder",      4096, NULL, 3,  NULL,            0); //core 0 
    //encoder might not need high priority if we consider speed is constant most of the time
    //xTaskCreatePinnedToCore(TaskPose,        "Pose",        4096, NULL, 3, &poseTask,    0); // core 1

    xTaskCreatePinnedToCore(TaskOdometryUnified_Stable, "OdomStable", 8192, NULL, 3, NULL, 0);

    //xTaskCreatePinnedToCore(TaskOdometry,    "Odometry",    4096, NULL, 3, NULL,         0); // core 0
    xTaskCreatePinnedToCore(TaskMotor,       "Motor",       4096, NULL, 2, &motorTask,   1); // core 1
    xTaskCreatePinnedToCore(TaskUltrasonic,  "Ultrasonic",  4096, NULL, 2, &ultrasonicTask,1); // core 1
    
    xTaskCreatePinnedToCore(TaskPurePursuit, "PurePursuit", 4096, NULL, 1, &pursuitTask, 1); // core 1


}

void loop() { 
}