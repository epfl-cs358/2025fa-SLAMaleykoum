#include "tasks/task_odometry.h"
#include "global_state.h"
#include "config.h"
#include "hardware/I2C_mutex.h"
#include "hardware/ImuSensor.h"
#include "hardware/EncoderCarVelocity.h"
#include "common/esp_link.h"
#include "common/data_types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Helper specific to this task
static float getYawIMU(const IMUData& imu_data) {
  float qw = imu_data.qw;
  float qx = imu_data.qx;
  float qy = imu_data.qy;
  float qz = imu_data.qz;
  float siny_cosp = 2.0 * (qw * qz + qx * qy);
  float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return atan2(siny_cosp, cosy_cosp);
}

static float normalizeAngleDiffRad(float diff) {
    if (diff > M_PI)  diff -= 2.0f * M_PI;
    if (diff < -M_PI) diff += 2.0f * M_PI;
    return diff;
}


void TaskOdometryUnified_Stable(void *pvParameters) {
    uint32_t prevTime = micros();
    uint32_t lastPrintTime = 0;

    const float MAX_YAW_CHANGE_RAD = 0.523599f;//0.2618f; //30 this is 15 degreees

    // Variable to track previous yaw and position
    float prevYaw = 0.0f;
    //float prevPosX = 0.0f;
    //float prevPosY = 0.0f;

    bool firstRun = true;


    for (;;) {
        uint32_t now = micros();
        float dt = (now - prevTime) / 1000000.0f;
        prevTime = now;
        
        //get values from IMU and Encoder
        i2c_lock();
        imu.readAndUpdate();
        IMUData imuData = imu.data();
        float currentYaw = getYawIMU(imuData);
        float currentVel = encoder.getWheelLinearVelocity();
        i2c_unlock();

        if (firstRun) {
            prevYaw = currentYaw;
            firstRun = false;
        }

        // Check if yaw change is too large, if so, skip the update
        float yawDiff = fabsf(normalizeAngleDiffRad((currentYaw - prevYaw)));

       
        if (yawDiff > MAX_YAW_CHANGE_RAD) {
            currentYaw = prevYaw;
        }
        
        //here should be a verification of yaw and velocity to check for big jumps, bu

        
        yaw = currentYaw;
        velocity = currentVel;
        
        if (xSemaphoreTake(poseMutex, portMAX_DELAY)) {
            posX -= velocity * sinf(yaw) * dt;
            posY += velocity * cosf(yaw) * dt;
            xSemaphoreGive(poseMutex);
        }

        prevYaw = yaw;

        //check if position wrong don't send, contrain it
        //if between each iteration no position chagnes of more than 5 cm from previous position

        //keep a state of angle of car, if it turns
        //avoid jumps with yaw with angle of yaw, so if previous yaw isn't 0 you skip 
        //check delta yaw and smooth, so between 2 iterations of odometry it's impossible for the car to move of let's say 20 degrees or let's say 45 degrees
        Pose2D pose;
        pose.x = posX;
        pose.y = posY;
        pose.theta = yaw;
        pose.timestamp_ms = millis();
        
        
        // if(!isPerformingCreneau){
        esp_link.sendPos(pose);
        // } 
        
        if (millis() - lastPrintTime > 1000) {
            Serial.printf("Pose TX: X=%.3f Y=%.3f θ=%.3f | 100Hz\n", posX, posY, yaw);
            lastPrintTime = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/*
#include "tasks/task_odometry.h"
#include "global_state.h"
#include "config.h"
#include "hardware/I2C_mutex.h"
#include "hardware/ImuSensor.h"
#include "hardware/EncoderCarVelocity.h"
#include "common/esp_link.h"
#include "common/data_types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Helper specific to this task
static float getYawIMU(const IMUData& imu_data) {
  float qw = imu_data.qw;
  float qx = imu_data.qx;
  float qy = imu_data.qy;
  float qz = imu_data.qz;
  float siny_cosp = 2.0 * (qw * qz + qx * qy);
  float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return atan2(siny_cosp, cosy_cosp);
}

// Helper to normalize angle difference to [-PI, PI]
static float normalizeAngleDiff(float angle_diff) {
    while (angle_diff > PI) angle_diff -= 2.0f * PI;
    while (angle_diff < -PI) angle_diff += 2.0f * PI;
    return angle_diff;
}

void TaskOdometryUnified_Stable(void *pvParameters) {
    uint32_t prevTime = micros();
    uint32_t lastPrintTime = 0;
    
    // State tracking for glitch detection
    static float prevYaw = 0.0f;
    static float prevVel = 0.0f;
    static float prevPosX = 0.0f;
    static float prevPosY = 0.0f;
    static bool firstRun = true;
    
    // Safety thresholds
    const float MAX_YAW_CHANGE_RAD = 15.0f * PI / 180.0f;  // 15 degrees per iteration (10ms)
    const float MAX_POS_JUMP_M = 0.05f;                     // 5 cm position jump
    const float MAX_VEL_CHANGE_MPS = 1.0f;                  // 1 m/s velocity jump (reasonable for 10ms)
    
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
        
        // check if computed position seems glitched compared to previous
        bool isValidReading = true;
        
        if (!firstRun) {
            // Check 1: Yaw jump detection
            float yawDiff = normalizeAngleDiff(currentYaw - prevYaw);
            if (fabs(yawDiff) > MAX_YAW_CHANGE_RAD) {
                Serial.printf("YAW GLITCH: %.1f° change rejected\n", yawDiff * 180.0f / PI);
                isValidReading = false;
            }
            
            // Check 2: Velocity jump detection
            float velDiff = fabs(currentVel - prevVel);
            if (velDiff > MAX_VEL_CHANGE_MPS) {
                Serial.printf("⚠️ VEL GLITCH: %.2f m/s change rejected\n", velDiff);
                isValidReading = false;
            }
            
            // Check 3: Position jump detection (predicted vs actual)
            // Calculate where we should be based on previous velocity
            float predictedPosX = prevPosX - prevVel * sinf(prevYaw) * dt;
            float predictedPosY = prevPosY + prevVel * cosf(prevYaw) * dt;
            
            // Calculate where we would be with new readings
            float newPosX = prevPosX - currentVel * sinf(currentYaw) * dt;
            float newPosY = prevPosY + currentVel * cosf(currentYaw) * dt;
            
            float posJump = sqrtf(powf(newPosX - predictedPosX, 2) + powf(newPosY - predictedPosY, 2));
            if (posJump > MAX_POS_JUMP_M) {
                Serial.printf("POS GLITCH: %.3fm jump rejected\n", posJump);
                isValidReading = false;
            }
        }
        
        // Update the state
        if (isValidReading || firstRun) {
            // Accept the reading
            yaw = currentYaw;
            velocity = currentVel;
            
            if (xSemaphoreTake(poseMutex, portMAX_DELAY)) {
                posX -= velocity * sinf(yaw) * dt;
                posY += velocity * cosf(yaw) * dt;
                
                // Store current state for next iteration
                prevPosX = posX;
                prevPosY = posY;
                xSemaphoreGive(poseMutex);
            }
            
            prevYaw = yaw;
            prevVel = velocity;
            firstRun = false;
            
            // Send valid pose
            Pose2D pose;
            if (xSemaphoreTake(poseMutex, portMAX_DELAY)) {
                pose.x = posX;
                pose.y = posY;
                xSemaphoreGive(poseMutex);
            }
            pose.theta = yaw;
            pose.timestamp_ms = millis();
            
            esp_link.sendPos(pose);
            
        } else {
            // Reject the reading - keep previous values
            // This allows the car to continue with last known good state
            Serial.println("Sensor reading rejected, using previous state");
        }
        
        if (millis() - lastPrintTime > 1000) {
            Serial.printf("Pose TX: X=%.3f Y=%.3f θ=%.3f V=%.2f | 100Hz\n", 
                          prevPosX, prevPosY, prevYaw, prevVel);
            lastPrintTime = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}*/