/**
 * @file DMS15.h
 * @brief Header file for DMS15 servo motor control class.
 *
 * @details
 *  This module provides public functions for initializing and
 *  controlling a DMS15 servo that orients the wheels.
 */
#ifndef DMS15_H
#define DMS15_H

#include <Arduino.h>
#include <ESP32Servo.h>

class DMS15 {
public:
    /**
     * @brief Constructor of a servo
     */
    DMS15(int pin);

    /**
     * @brief Initialises servo 
     * 
     * Attach the pin to the hardware.
     * Set the servo to 110 angle
     * 
     * @return true if it's ready to move.
     */
    bool begin();

    /**
     * @brief Manually set servo to specific angle between 0 and 180 degrees.
     */
    void setAngle(int angle);

    /**
     * @brief gives the current angle
     * @return the current angle of the servo.
     */
    int getAngle() const {return currentAngle;}

private:
    Servo servo;
    int servoPin;
    int currentAngle;
};

#endif