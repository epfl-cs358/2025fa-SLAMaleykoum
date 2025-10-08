#include "NEO6M.h"

NEO6M::NEO6M(HardwareSerial &ser, uint32_t baud, int8_t rxPin, int8_t txPin)
  : gpsSerial(ser), baud(baud), rxPin(rxPin), txPin(txPin) {}

bool NEO6M::begin() {
  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
  delay(1000);
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    if (gpsSerial.available()) {
      Serial.println("GPS found");
      return true;
    }
  }
  return false;
}

bool NEO6M::read() {
  bool updated = false;
  unsigned long timeout = millis() + 3000;
  
  while (millis() < timeout) {
    if(gpsSerial.available()) {
      char c = gpsSerial.read();
      Serial.print(c);
      gps.encode(c);
      if (gps.location.isUpdated()) {
        updated = true;
      }
    }
  }
  return updated;
}

void NEO6M::populateNavSatFix(sensor_msgs__msg__NavSatFix &msg) {
  msg.latitude  = gps.location.lat();
  msg.longitude = gps.location.lng();
  msg.altitude  = gps.altitude.meters();
  msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
  msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
  for (int i = 0; i < 9; i++) msg.position_covariance[i] = 0.0;
  msg.position_covariance_type = 
      sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
}
