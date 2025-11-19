#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "common/data_types.h"   // for IMUData

class Odometry {
public:
    Odometry();

    void reset();

    // Main update: IMU + encoder velocity
    void update(const IMUData& imu, float velocity);

    float x() const { return posX; }
    float y() const { return posY; }
    float yaw() const { return yawFiltered; }

private:
    float computeYaw(const IMUData& imu);

    float posX, posY;
    float yawFiltered;
    unsigned long lastTime;
};

#endif
