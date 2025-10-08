/* MotorController.h */
#pragma once
#include <ESP32Servo.h>

static constexpr uint16_t NEUTRAL_US = 1500;
static constexpr uint16_t MAX_FORWARD_US = 2000;
static constexpr uint16_t MAX_REVERSE_US = 1000;

// number of microseconds to wait between each step
static constexpr uint16_t RAMP_STEP_US = 20; 
// number of microseconds to wait between each step
static constexpr uint16_t UPDATE_PERIOD_MS = 50; // 50ms between each step

class MotorController {
public:
  MotorController(int pwmPin);

  /// Call once in setup(); returns false on failure
  bool begin();

  void setTargetUs(uint16_t us);

  void setTargetPercent(float p);

  void emergencyStop();

  void update();

  uint16_t currentUs() const { return _currentUs;}

  void command(int dir);

private:
  int   _pwmPin;
  Servo _esc;
  uint16_t _targetUs = NEUTRAL_US; // target pulse width in microseconds
  uint16_t _currentUs = NEUTRAL_US; // current pulse width in microseconds
};