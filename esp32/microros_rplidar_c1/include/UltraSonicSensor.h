#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>
#include <sensor_msgs/msg/range.h>

class UltraSonicSensor {
public:
    UltraSonicSensor(uint8_t triggerPin, uint8_t echoPin);

    bool begin();
    float readDistance();
    sensor_msgs__msg__Range range_msg;
    static constexpr float STOP_THRESHOLD = 0.50f;

private:
    uint8_t trigPin;
    uint8_t echoPin;
};

#endif
