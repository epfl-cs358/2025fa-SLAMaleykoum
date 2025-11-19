#include "Odometry.h"
#include <cmath>

Odometry::Odometry() {
    reset();
}

void Odometry::reset() {
    posX = 0.0f;
    posY = 0.0f;
    yawFiltered = 0.0f;
    lastTime = millis();
}

// Convert quaternion → yaw
float Odometry::computeYaw(const IMUData& imu) {
    float qw = imu.qw;
    float qx = imu.qx;
    float qy = imu.qy;
    float qz = imu.qz;

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

    return atan2(siny_cosp, cosy_cosp);
}

void Odometry::update(const IMUData& imu, float velocity) {

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    // Compute yaw from IMU quaternion
    float yawRaw = computeYaw(imu);

    // Low-pass filter yaw
    const float alpha = 0.15f;
    yawFiltered = alpha * yawRaw + (1.0f - alpha) * yawFiltered;

    // Dead reckoning integration
    posX -= velocity * sinf(yawFiltered) * dt;
    posY += velocity * cosf(yawFiltered) * dt;
}
