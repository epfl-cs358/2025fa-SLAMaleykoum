/**
 * @file HR-SR04.cpp
 * @brief Implementation of the UltraSonicSensor class for the HR-SE04 sensor
 * 
 * @author SLAMaleykoum & TurboSLAM : Same file as the TurboSLAM but removed all code related to ROS messages
 * @date Oct 2025
 */

#include "hardware/UltraSonicSensor.h"


/**
 * @brief Constructor that creates an UltraSonicSensor object that stores which pins are used by the sensor 
*/
UltraSonicSensor::UltraSonicSensor(uint8_t triggerPin, uint8_t echoPin)
    : trigPin(triggerPin), echoPin(echoPin) {}

/**
 * @brief Initializes the sensor by configurating the pins and verifying that it can receive echos 
 * @return true if it receives an echo and false if it doesn't 
*/    
bool UltraSonicSensor::begin() {

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    digitalWrite(trigPin, LOW);
    delay(50);

    /*Sends pulse*/
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    /*Checks how long it took for the pulse to be received
    *Prints message accordingly
    */
    unsigned long duration = pulseIn(echoPin, HIGH, 38UL * 1000UL); 
    if (duration == 0) {
        // no echo received
        Serial.println("Ultrasonic begin: no echo (sensor not detected?)");
        return false;
    }

    Serial.printf("Ultrasonic begin: got echo pulse %lu µs\n", duration);
    return true;
}

/**
 * @brief Measures the distance between the sensor and an object
 * @return -1.0 if no echo is received (no object or a timeout) or the distance in meters
 */
float UltraSonicSensor::readDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 25000); // timeout 25ms
    float distance = duration * 0.0343 / 2.0;

    if (duration == 0) {
        return -1.0; // timeout or no object
    }
    
    return distance / 100.0; // convert to meters
}
