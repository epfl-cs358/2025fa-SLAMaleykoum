#include <ImuSensor.h>
#include <Wire.h>
#include <I2C_wire.h>

sensor_msgs__msg__Imu imu_msg;
bool ImuSensor::begin() {

  Serial.println("\nI2C Scanner imu");
  for (byte address = 1; address < 127; address++) {
    I2C_wire.beginTransmission(address);
    if (I2C_wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.println("I2C Scanner imu done");

  if (!bno086.begin_I2C(0x4B, &I2C_wire)) { 
    Serial.println("Failed to initialize BNO08x!");
    return false;
  }
  Serial.println("BNO086initialized");
  if (!configureSensor()) {
    Serial.println("Sensor configuration failed!");
    return false;
  }

  // Initialize message frame
  imu_msg.header.frame_id.data = (char *)"imu_frame";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  return true;
}

bool ImuSensor::configureSensor() {
  return bno086.enableReport(SH2_ROTATION_VECTOR, 400) &&
  bno086.enableReport(SH2_LINEAR_ACCELERATION, 400) &&
  bno086.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 400);
}

void ImuSensor::readAndUpdate() {
  if (!bno086.wasReset() && !bno086.getSensorEvent(&sensorValue)) return;

  imu_msg.header.stamp.sec = millis() / 1000;
  imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

  imu_msg.orientation = {0};
  imu_msg.angular_velocity = {0};
  imu_msg.linear_acceleration = {0};

  // Orientation quaternion
  if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
    imu_msg.orientation.w = sensorValue.un.rotationVector.real;
    imu_msg.orientation.x = sensorValue.un.rotationVector.i;
    imu_msg.orientation.y = sensorValue.un.rotationVector.j;
    imu_msg.orientation.z = sensorValue.un.rotationVector.k;
  }

  // Linear acceleration (m/s²)
  if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
    imu_msg.linear_acceleration.x = sensorValue.un.linearAcceleration.x;
    imu_msg.linear_acceleration.y = sensorValue.un.linearAcceleration.y;
    imu_msg.linear_acceleration.z = sensorValue.un.linearAcceleration.z;
  }

  // Angular velocity (rad/s)
  if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
    imu_msg.angular_velocity.x = sensorValue.un.gyroscopeUncal.x;
    imu_msg.angular_velocity.y = sensorValue.un.gyroscopeUncal.y;
    imu_msg.angular_velocity.z = sensorValue.un.gyroscopeUncal.z;
  }
}

