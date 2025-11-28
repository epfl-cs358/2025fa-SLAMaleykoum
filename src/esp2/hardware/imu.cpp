#include "ImuSensor.h"
#include <Wire.h>
#include "I2C_wire.h"
#include <Arduino.h>

//ImuSensor::ImuSensor(int sdaPin, int sclPin) : 
  //  bno086(), sensorValue{}, imu_data{}, _sdaPin(sdaPin), _sclPin(sclPin) {}
ImuSensor::ImuSensor()
    : bno086(), sensorValue{}, imu_data{} {}

bool ImuSensor::begin() {
    //I2C_wire.begin(_sdaPin, _sclPin);

    // Initialize the BNO086
    //if (!bno086.begin_I2C(IMU_ADDR, &I2C_wire, 4)) { 
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
    return bno086.enableReport(SH2_GAME_ROTATION_VECTOR, period_us) 
            && bno086.enableReport(SH2_LINEAR_ACCELERATION, period_us);
            //&& bno086.enableReport(SH2_GYROSCOPE_UNCALIBRATED, period_us);
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

    // Linear acceleration (m/s²)
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        const auto& a = sensorValue.un.linearAcceleration;
        imu_data.acc_x = a.x;
        imu_data.acc_y = a.y;
        imu_data.acc_z = a.z;
    }

    // Angular velocity (rad/s)
    // if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
    //     const auto& g = sensorValue.un.gyroscope;
    //     imu_data.omega_x = g.x;
    //     imu_data.omega_y = g.y;
    //     imu_data.omega_z = g.z;
    // }

    imu_data.timestamp_ms = millis();
}
