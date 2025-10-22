/**
 * @file manual_main.cpp
 * @brief Main program for manually controlling the car via keyboard inputs (WDAS Q)
 * 
 * @author SLAMaleykoum
 * @date October 2025
 */


#include <Arduino.h>
#include "MotorManager.h"
#include "DMS15.h"
#include "UltraSonicSensor.h"

//TODO double check before runnign that all the pins are correct via the car

// Pins used by the hardware components on the esp 2
#define ESC_PIN 15 //pin used for the motor
#define SERVO_DIR_PIN 6 //the servo that modifies the direction of the wheels
#define US_TRIG_PIN 10 //previous group didn't use as constants
#define US_ECHO_PIN 11

//Harwdare objects
MotorManager motor(ESC_PIN); //motor
DMS15 servo_dir(SERVO_DIR_PIN); //servo
UltraSonicSensor ultrasonic(US_TRIG_PIN, US_ECHO_PIN); //ultra sonic sensor
// Parameters
int drivePower = 50;      
int angle = 90;
const int stepDeg = 5;
const int minAngle = 60;
const int maxAngle = 120;

void setup() {
    Serial.begin(115200);
    while (!Serial) ;

    //Initialize motor
    if (!motor.begin()) {
        Serial.println("Motor init failed!");
        while (true);
    }

    //Initialize servo
    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }
    servo_dir.setAngle(angle);

    //Initialize ultrasonic sensor
    if (!ultrasonic.begin()) {
        Serial.println("Ultrasonic init failed!");
        while (true);
    }

    Serial.println("Use W/S to drive, SPACE to stop, A/D to steer, Q to quit.");
}

void loop() {
    float dist = ultrasonic.readDistance(); //read once per loop so no multiple actions at the same time

    //read keyboard serial input
    if (Serial.available() > 0) {
        char c = tolower(Serial.read());

        switch (c) {

            //Forward
            case 'w':
                if (dist < 0 || dist > UltraSonicSensor::STOP_THRESHOLD) {
                    motor.forward(drivePower / 100.0f);
                    Serial.println("Forward");
                } else {
                    motor.stop();
                    Serial.println("Obstacle! Stop");
                }
                break;
            
            //Backwards
            case 's':
                motor.backward(drivePower / 100.0f);
                Serial.println("Backward");
                break;
            
            //Stop
            case ' ':
                motor.stop();
                Serial.println("Stop");
                break;
            
            //Turn left    
            case 'a':
                angle = max(minAngle, angle - stepDeg);
                servo_dir.setAngle(angle);
                Serial.print("Steer Left: "); Serial.println(angle);
                break;

            //Turn right
            case 'd':
                angle = min(maxAngle, angle + stepDeg);
                servo_dir.setAngle(angle);
                Serial.print("Steer Right: "); Serial.println(angle);
                break;
            
            //Quit
            case 'q':
                motor.stop();
                Serial.println("Quit");
                while (true);
        }
    }

    //Emergency stop if an obstacle is detected
    if (dist > 0 && dist < UltraSonicSensor::STOP_THRESHOLD) {
        motor.stop();
        Serial.println("Emergency Stop! Obstacle detected.");
    }

    motor.update();
    delay(10);
}
