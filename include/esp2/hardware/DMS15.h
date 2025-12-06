#ifndef DMS15_H
#define DMS15_H

#include <Arduino.h>
#include <ESP32Servo.h>

class DMS15 {
public:
    DMS15(int pin);
    bool begin();
    void setAngle(int angle);
    // Used to tilt the lidar
    void tiltLidar(float angleMin, float angleMax, unsigned long T_ms);
    //void tiltLidar();
    int getAngle() const;

private:
    Servo servo;
    int servoPin;
    int currentAngle;
};

#endif