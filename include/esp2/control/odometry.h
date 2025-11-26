#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "common/data_types.h" 

class Odometry {
public:
    Odometry();

    void reset();

    void update(const IMUData& imu, float velocity);

    float x() const { return posX; }
    float y() const { return posY; }
    float yaw() const { return yawFiltered; }

    float getX(float X0, float velocity, float yaw, float dt);
    float getY(float Y0, float velocity, float yaw, float dt);

    float computeYaw(const IMUData& imu);

private:
    

    float posX, posY;
    float yawFiltered;
    unsigned long lastTime;
};

#endif
