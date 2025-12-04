/**
 * @file MotorController.cpp
 * @brief Implementation of MotorController class to control the brushed DC motor via the Tamiya ESC.
 * 
 * @author SLAMaleykoum & TurboSLAM 
 * @date Oct 2025
 */

#include "hardware/MotorController.h"

/**
 * @brief Construct a new MotorController object
 * @param pwmPin Pin connected to ESC signal input
 */
MotorController::MotorController(int pwmPin)
  : _pwmPin(pwmPin) {}

  /// Call in setup(), returns false on pin-mode failure
  bool MotorController::begin() {
    _esc.setPeriodHertz(50); // Set frequency to 50 Hz
    _esc.attach(_pwmPin, MAX_REVERSE_US, MAX_FORWARD_US); // Attach the ESC to the PWM pin
    if (!_esc.attached()) {return false;}

    // ESC calibration: full forward 3s -> neutral 3s
    // _esc.writeMicroseconds(MAX_FORWARD_US);  
    // delay(3500); 
    _esc.writeMicroseconds(NEUTRAL_US);  
    delay(3500);

    // Initialize internal state
    _currentUs = NEUTRAL_US; 
    _targetUs = NEUTRAL_US;

    return true;
  }

  void MotorController::setTargetUs(uint16_t us) {
    // Set the target pulse width in microseconds
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
    // Gradually change speed until target is reached
    if (_currentUs == _targetUs) { return; }
     
    if (_currentUs < _targetUs) {
      _currentUs = min<uint16_t>(_currentUs + RAMP_STEP_US, _targetUs);
    } else {
      _currentUs = max<uint16_t>(_currentUs - RAMP_STEP_US, _targetUs);
    }
    _esc.writeMicroseconds(_currentUs);
  }

  // /// Drive the motor: +1 forward, -1 reverse, 0 stop
  // void MotorController::command(int dir) {
  //   // Convert direction to microseconds
  //   int targetUs = dir;
  //   int currentUs = _esc.readMicroseconds();
  //   const int step = 5; // Adjust step size for smoother acceleration/deceleration
  //   // Gradually change speed until target is reached
  //   while (currentUs != targetUs) {
  //     if (currentUs < targetUs) {
  //       currentUs = min(currentUs + step, targetUs);
  //     } else {
  //       currentUs = max(currentUs - step, targetUs);
  //     }
  //     _esc.writeMicroseconds(currentUs);
  //     delay(50); // Small delay for stability
  //   }
  // }
