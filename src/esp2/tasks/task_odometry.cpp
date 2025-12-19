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

    const float MAX_YAW_CHANGE_RAD = 0.2618f; //30 this is 15 degreees

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
        
        esp_link.sendPos(pose);
        
        if (millis() - lastPrintTime > 1000) {
            Serial.printf("Pose TX: X=%.3f Y=%.3f θ=%.3f | 100Hz\n", posX, posY, yaw);
            lastPrintTime = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}