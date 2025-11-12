/**
 * @file UltraSonicSensor.h
 * @brief Header file defining the UltraSonicSensor class to be used to measure the distance between 
 * the car and obstacles
 *
 * @author SLAMaleykoum & TurboSLAM : Same file as the TurboSLAM but removed all code related to ROS messages
 * @date Oct 2025
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>


/**
 * @class UltraSonicSensor 
 * @brief Provides the distance between the HR-SR04 ultra sonic sensor and an object
*/
class UltraSonicSensor {
public:

    /**
     * @brief Construct an Ultra sonic sensor object
     * @param triggerPin pin used to send an ultrasonic trigger pulse
     * @param echoPin pin used to receive the echo pulse
     */
    UltraSonicSensor(uint8_t triggerPin, uint8_t echoPin);

    /**
     * @brief initializes the sensor to verify that it can send and receive a pulse 
     * @return true if it receives an echo and false if it doesn't 
     */
    bool begin();

    /**
     * @brief Measures the distance between the sensor and an object
     * @return -1.0 if no echo is received (no object or a timeout) or the distance in meters
     */
    float readDistance();

    /**
     * @brief safe distance threshold to an object else trigger an emergency stop
     */
    static constexpr float STOP_THRESHOLD = 0.50f; //safety constant of 0.5 meters

private:
    /*Pins to connect sensor to board*/
    uint8_t trigPin; //connected to the sensor trigger input
    uint8_t echoPin; //connected to the sensor echo output
};

#endif
