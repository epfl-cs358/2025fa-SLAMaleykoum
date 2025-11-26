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

float Odometry::computeYaw(const IMUData& imu_data) {
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

float Odometry::getX(float X0, float velocity, float yaw, float dt) {
    //posX -= velocity * sinf(yaw) * dt;
    
    //posY += velocity * cosf(yaw) * dt;

    return (X0 - velocity * sinf(yaw) * dt);
}


float Odometry::getY(float Y0, float velocity, float yaw, float dt) {
    

    return (Y0 + velocity * cosf(yaw) * dt);



    
/*
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    float yawRaw = computeYaw(imu);

    yawFiltered = yawRaw;


    //const float alpha = 0.15f;
    //yawFiltered = alpha * yawRaw + (1.0f - alpha) * yawFiltered;

  
    posX -= velocity * sinf(yawFiltered) * dt;
    posY += velocity * cosf(yawFiltered) * dt;*/


}

void Odometry::update(const IMUData& imu, float velocity) {

    

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    float yawRaw = computeYaw(imu);

    yawFiltered = yawRaw;


    //const float alpha = 0.15f;
    //yawFiltered = alpha * yawRaw + (1.0f - alpha) * yawFiltered;

  
    posX -= velocity * sinf(yawFiltered) * dt;
    posY += velocity * cosf(yawFiltered) * dt;
}


