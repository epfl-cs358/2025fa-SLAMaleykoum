// #include "AS5600Encoder.h"
// #include "I2C_mutex.h"
// #include "I2C_wire.h"

// AS5600Encoder::AS5600Encoder(int sdaPin, int sclPin) : 
//     _sdaPin(sdaPin), _sclPin(sclPin) {}

// bool AS5600Encoder::begin() {
//     I2C_wire.begin(_sdaPin, _sclPin, 400000); // 400 kHz ? ---------

//     I2C_wire.beginTransmission(AS5600_ADDR);
//     if (I2C_wire.endTransmission() != 0) {
//         Serial.println("AS5600 not responding at 0x36");
//         return false;
//     }

//     // test the sensor
//     if (getRawAngle() == 0xFFFF) {
//         Serial.println("AS5600 read failed");
//         return false;
//     }

//     return true;
// }

// uint16_t AS5600Encoder::getRawAngle() {

//     // protect the i2c bus from other connections (if occupied, returns 0xFFFF)
//     if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
//         return 0xFFFF; 
//     }

//     I2C_wire.beginTransmission(AS5600_ADDR);
//     I2C_wire.write(RAW_ANGLE_REG);
//     if (I2C_wire.endTransmission(false) != 0) {
//         xSemaphoreGive(i2c_mutex);
//         return 0xFFFF; 
//     }

//     if (I2C_wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) {
//         xSemaphoreGive(i2c_mutex);
//         return 0xFFFF; 
//     }

//     uint8_t msb = I2C_wire.read();
//     uint8_t lsb = I2C_wire.read();
//     xSemaphoreGive(i2c_mutex);
//     uint16_t raw = ((uint16_t)msb << 8) | lsb;
//     return raw & 0x0FFF;  
// }

// float AS5600Encoder::update() {
//     uint16_t raw = getRawAngle();

//     if (raw == 0xFFFF) {
//         return -1.0f;  // error
//     }

//     return (raw * 360.0f) / 4096.0f;  // angle in degrees
// }
