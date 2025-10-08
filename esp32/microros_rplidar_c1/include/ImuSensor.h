#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Adafruit_BNO08x.h>
#include "sensor_msgs/msg/imu.h"

extern sensor_msgs__msg__Imu imu_msg;

class ImuSensor {
    public:
        bool begin();
        void readAndUpdate();
        sensor_msgs__msg__Imu& getMsg() { return imu_msg; }

    private:
        Adafruit_BNO08x bno086;
        sh2_SensorValue_t sensorValue;

    bool configureSensor();
    };

extern ImuSensor imuSensor;

#endif