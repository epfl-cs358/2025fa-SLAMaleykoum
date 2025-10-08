#ifndef NEO6M_H
#define NEO6M_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <rcl/rcl.h>

class NEO6M {
public:
  NEO6M(HardwareSerial &ser, uint32_t baud, int8_t rxPin, int8_t txPin);
  bool begin();
  bool read();
  void populateNavSatFix(sensor_msgs__msg__NavSatFix &msg);

private:
  HardwareSerial &gpsSerial;
  TinyGPSPlus    gps;
  uint32_t       baud;
  int8_t         rxPin, txPin;
};

#endif 
