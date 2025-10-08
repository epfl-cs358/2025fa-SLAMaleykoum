#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>
#include "std_msgs/msg/int32.h"

class AS5600Encoder {
    public:
        bool begin();
        std_msgs__msg__Int32 update(); 

    private:
        const uint8_t AS5600_ADDR = 0x36;
        const uint8_t RAW_ANGLE_REG = 0x0C;
        uint16_t getRawAngle();
};

extern AS5600Encoder encoder;

#endif
