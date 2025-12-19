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

namespace Control {


void performMultiPointTurn() {
        const float ANGLE_TOLERANCE = 5.0; //in degrees
        const int MAX_ITERATIONS = 5; //from testing 5 is around 180 degrees so double should return facing the original direction

        //Start by moving backwards to have enough space to turn around without crashing into the wall
        servo_dir.setAngle(90);
        
        uint32_t start = millis();
        while (millis() - start < 100) {}
        for (int i = 0; i < 10; i++) {
            motor.backward(0.25f);
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
            while (millis() - start < 100) {}
    
            for (int i = 0; i < 8; i++) {
                motor.backward(0.25f);
                motor.update();
            }
            
            start = millis();
            while (millis() - start < 300) {}

            //stop for pause
            for(int i = 0; i<3; i++){
                motor.stop();
                motor.update();
            }
            
            start = millis();
            while (millis() - start < 200) {}
            
            //Turn wheels opposite direction and move forward
            servo_dir.setAngle(60);
            start = millis();
            while (millis() - start < 100) {}
            
            for (int i = 0; i < 8; i++) {
                motor.forward(0.17f);
                motor.update();
            }
            
            start = millis();
            while (millis() - start < 500) {}

            //Stop break
            for(int i = 0; i<3; i++){
                motor.stop();
                motor.update();
            }
            
            start = millis();
            while (millis() - start < 200) {}
        }
        
        //reached target so set wheels stright to face target
        servo_dir.setAngle(90);
        for(int i = 0; i<3; i++){
            motor.stop();
            motor.update();
        }
        
        uint32_t start1 = millis();
        while (millis() - start1 < 200) {}
        
    }
}
