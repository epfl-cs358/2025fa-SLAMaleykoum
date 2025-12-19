/**
 * @file recovery_maneuvers.cpp
 * 
 * @author SLAMaleykoum
 * @date Nov 2025
 */

#include "control/recovery_maneuvers.h"
#include "esp2/global_state.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include <math.h>
#include "esp2/config.h"

namespace Control {

void performMultiPointTurn() {
        const float ANGLE_TOLERANCE = 5.0; //in degrees
        const int MAX_ITERATIONS = 2; //from testing 5 is around 180 degrees, 2 is around 35 degrees
        const float BACKWARD_MOTOR_DRIVE = 0.25f;

        //Start by moving backwards to have enough space to turn around without crashing into the wall
        servo_dir.setAngle(Config::STRAIGHT_ANGLE);
        uint32_t start = millis();
        while (millis() - start < 100) {} //pause for 100 ms

        for (int i = 0; i < 10; i++) {
            motor.backward(BACKWARD_MOTOR_DRIVE);
            motor.update();
        }

        start = millis();
        while (millis() - start < 300) {}
        for(int i = 0; i<3; i++){
            motor.stop();
            motor.update();
        }

        start = millis();
        while (millis() - start < 200) {}
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
            
            float currentYaw = currentPose.theta * 180.0 / PI;
            
            //Start rotation backwards while wheels are turned towards one direction
            servo_dir.setAngle(120);
            start = millis();
            while (millis() - start < 100) {} // delay of 100ms
           
            for (int i = 0; i < 8; i++) {
                motor.backward(BACKWARD_MOTOR_DRIVE);
                motor.update();
            }
            
            start = millis();
            while (millis() - start < 300) {}  // delay of 300ms

            //stop for pause
            for(int i = 0; i<3; i++){
                motor.stop();
                motor.update();
            }
            
            start = millis();
            while (millis() - start < 200) {}  // delay of 200ms
            
            //Turn wheels opposite direction and move forward
            servo_dir.setAngle(60);
            start = millis();
            while (millis() - start < 100) {}  // delay of 100ms
            //delay(100);
            for (int i = 0; i < 8; i++) {
                motor.forward(Config::MOTOR_POWER_DRIVE);
                motor.update();
            }

            start = millis();
            while (millis() - start < 500) {}  // delay of 500ms

            //Stop break
            for(int i = 0; i<3; i++){
                motor.stop();
                motor.update();
            }
            start = millis();
            while (millis() - start < 200) {}  // delay of 200ms
        }
        
        //reached target so set wheels stright to face target
        servo_dir.setAngle(Config::STRAIGHT_ANGLE);
        for(int i = 0; i<3; i++){
            motor.stop();
            motor.update();
        }
        uint32_t start1 = millis();
        while (millis() - start1 < 200) {}  // delay of 200ms
        
    }
}
