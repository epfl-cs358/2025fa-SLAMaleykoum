#include "hardware/ImuSensor.h"
#include <Wire.h>
#include "hardware/I2C_wire.h"
#include <Arduino.h>

ImuSensor::ImuSensor()
    : bno086(), sensorValue{}, imu_data{} {}

bool ImuSensor::begin() {
    // Initialize the BNO086
    if (!bno086.begin_I2C(IMU_ADDR, &I2C_wire)) {
        Serial.println("Failed to initialize BNO08x!");
        return false;
    }
    Serial.println("BNO086 initialized");

    // Enable the data reports (quaternion + accel + gyro)
    if (!configureSensor()) {
        Serial.println("Sensor configuration failed!");
        return false;
    }

    return true;
}

bool ImuSensor::configureSensor(uint32_t period_us) {
    return bno086.enableReport(SH2_GAME_ROTATION_VECTOR, period_us);
}

void ImuSensor::readAndUpdate() {
    // If there was no reset and no event available, return
    if (!bno086.wasReset() && !bno086.getSensorEvent(&sensorValue)) return;

    // Orientation quaternion
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        const auto& rv = sensorValue.un.gameRotationVector;
        imu_data.qw = rv.real;
        imu_data.qx = rv.i;
        imu_data.qy = rv.j;
        imu_data.qz = rv.k;
    }

    imu_data.timestamp_ms = millis();
}
