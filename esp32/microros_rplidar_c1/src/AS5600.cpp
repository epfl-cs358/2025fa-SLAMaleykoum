#include "AS5600Encoder.h"
#include "I2C_mutex.h"
#include "I2C_wire.h"

bool AS5600Encoder::begin() {
    Serial.println("\nI2C Scanner encoder");
    for (byte address = 1; address < 127; address++) {
        I2C_wire.beginTransmission(address);
        if (I2C_wire.endTransmission() == 0) {
            Serial.print("I2C device found at address 0x");
            Serial.println(address, HEX);
        }
    }
    return true;
}

uint16_t AS5600Encoder::getRawAngle() {

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return 0xFFFF; 
    }

    I2C_wire.beginTransmission(AS5600_ADDR);
    I2C_wire.write(RAW_ANGLE_REG);
    if (I2C_wire.endTransmission(false) != 0) {
        xSemaphoreGive(i2c_mutex);
        return 0xFFFF; 
    }

    if (I2C_wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) {
        xSemaphoreGive(i2c_mutex);
        return 0xFFFF; 
    }

    uint8_t msb = I2C_wire.read();
    uint8_t lsb = I2C_wire.read();
    xSemaphoreGive(i2c_mutex);
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    return raw & 0x0FFF;  
}

std_msgs__msg__Int32 AS5600Encoder::update() {
    std_msgs__msg__Int32 msg;
    uint16_t raw = getRawAngle();

    if (raw != 0xFFFF) {
        float angle_deg = (raw * 360.0f) / 4096.0f;
        msg.data = static_cast<int32_t>(angle_deg);  
    } else {
        msg.data = -1;
    }

    return msg;
}
