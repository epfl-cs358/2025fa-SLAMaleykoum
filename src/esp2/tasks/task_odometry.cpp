#include "tasks/task_odometry.h"
#include "global_state.h"
#include "config.h"
#include "hardware/I2C_mutex.h"
#include "hardware/ImuSensor.h"
#include "hardware/EncoderCarVelocity.h"
#include "common/esp_link.h"
#include "common/data_types.h"

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
