/**
 * @file MotorController.h
 * @brief Low-level driver for controlling the DC motor through the Tamiya brushed ESC
 * 
 * @author SLAMaleykoum & TurboSLAM no changes
 * @date Oct 2025
 */

#pragma once
#include <ESP32Servo.h>

//Neutral pulse width 
static constexpr uint16_t NEUTRAL_US = 1500;
//Maximum forward pulse width
static constexpr uint16_t MAX_FORWARD_US = 2000;
//Maximum backwards pulse width
static constexpr uint16_t MAX_REVERSE_US = 1000;

// number of microseconds to wait between each step
static constexpr uint16_t RAMP_STEP_US = 20; 
// number of microseconds to wait between each step
static constexpr uint16_t UPDATE_PERIOD_MS = 50; // 50ms between each step

/**
 * @class MotorController 
 * @brief Sends the PWM signals to the motor via the ESC
 */

class MotorController {
public:
  /**
   * @brief Construct a new MotorController object.
   * @param pwmPin Pin used to send the PWM signal to the ESC
   */
  MotorController(int pwmPin);

  /**
   * @brief Initializes the ESC and sets it to neutral.
   * @return True if ESC successfully attached to the PWM pin, false otherwise.
  */
  bool begin(); // Call once in setup(); returns false on failure

  /**
   * @brief Set a target PWM pulse width directly.
   * @param us Desired pulse width (in microseconds).
  */
  void setTargetUs(uint16_t us);

  /**
   * @brief Set the target power level as a percentage.
   * @param p Desired power level (-1.0 = full reverse, +1.0 = full forward).
   */
  void setTargetPercent(float p);

  /**
   * @brief Instantly stop the motor (sets PWM to neutral).
   */
  void emergencyStop();

  /**
   * @brief Smoothly update PWM toward the target value.
   *
   * Should be called periodically
   */
  void update();

  /**
   * @brief Get the current PWM pulse width being applied.
   * @return Current pulse width in microseconds.
   */
  uint16_t currentUs() const { return _currentUs;}

  
  //void command(int dir); TODO they didn't end up using it 

private:
  int   _pwmPin;
  Servo _esc;
  uint16_t _targetUs = NEUTRAL_US; // target pulse width in microseconds
  uint16_t _currentUs = NEUTRAL_US; // current pulse width in microseconds
};