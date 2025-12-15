/**
 * @file MotorController.cpp
 * @brief Implementation of MotorController class to control the brushed DC motor via the Tamiya ESC.
 */
#include "hardware/MotorController.h"

MotorController::MotorController(int pwmPin)
  : _pwmPin(pwmPin) {}

bool MotorController::begin() {
  _esc.setPeriodHertz(50);
  _esc.attach(_pwmPin, MAX_REVERSE_US, MAX_FORWARD_US);
  if (!_esc.attached()) return false;

  // ESC calibration: full forward 3s -> neutral 3s
  _esc.writeMicroseconds(NEUTRAL_US);  
  delay(3500);

  // Initialize internal state
  _currentUs = NEUTRAL_US; 
  _targetUs = NEUTRAL_US;

  return true;
}

void MotorController::setTargetUs(uint16_t us) {
  _targetUs = constrain(us, MAX_REVERSE_US, MAX_FORWARD_US);
}

void MotorController::setTargetPercent(float p) {
  // Set the target pulse width as a percentage of the range
  _targetUs = constrain(p, -1.0f, 1.0f);

  if (p >= 0.0f) {
    setTargetUs(NEUTRAL_US + uint16_t(p * (MAX_FORWARD_US - NEUTRAL_US)));
  } else {
    setTargetUs(NEUTRAL_US + uint16_t(p * (NEUTRAL_US - MAX_REVERSE_US)));
  }
}

void MotorController::emergencyStop(){
  if (_targetUs > NEUTRAL_US) setTargetUs(NEUTRAL_US);
}

void MotorController::update() {
  if (_currentUs == _targetUs) return;
    
  if (_currentUs < _targetUs) {
    _currentUs = min<uint16_t>(_currentUs + RAMP_STEP_US, _targetUs);
  } else {
    _currentUs = max<uint16_t>(_currentUs - RAMP_STEP_US, _targetUs);
  }
  _esc.writeMicroseconds(_currentUs);
}
