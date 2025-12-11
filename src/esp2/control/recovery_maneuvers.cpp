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


//TODO: play with the delays after the motor stops to make it faster and after changing the angle, 
//but may reduce precision of positon, ebfore 300 isntead of 100
namespace Control {

    float calculateTargetYaw(float currentX, float currentY, 
                            float targetX, float targetY) {
        float dx = targetX - currentX;
        float dy = targetY - currentY;

        float mathAngle = atan2(dy, dx) * 180.0 / PI;

        float targetYaw = mathAngle - 90.0;
        
        while (targetYaw > 180) targetYaw -= 360;
        while (targetYaw < -180) targetYaw += 360;
        
        return targetYaw;
    }

    float normalizeAngleDiff(float target, float current) {
        float diff = target - current;

        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    void performMultiPointTurn(float targetYaw) {
        const float ANGLE_TOLERANCE = 5.0; //in degrees
        const int MAX_ITERATIONS = 10; //from testing 5 is around 180 degrees so double should return facing the original direction

        //Start by moving backwards to have enough space to turn around without crashing into the wall
        servo_dir.setAngle(90);
        delay(100);
        for (int i = 0; i < 10; i++) {
            motor.backward(0.21f);
            motor.update();
        }
        delay(500);
        for(int i = 0; i<3; i++){
            motor.stop();
            motor.update();
        }
        delay(200);
        
        for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
            
            float currentYaw = currentPose.theta * 180.0 / PI;
            
            float error = normalizeAngleDiff(targetYaw, currentYaw);
            
            //Check if we've reached target angle
            if (abs(error) < ANGLE_TOLERANCE) {
                break;
            }
            
            //Determine turn direction:The target is to my LEFT, so I need to rotate my body LEFT (counterclockwise) to face it"
            bool turnLeft = (error > 0); 
    
            //Start rotation backwards while wheels are turned towards one direction
            servo_dir.setAngle(turnLeft ?120:60);
            delay(100);
            for (int i = 0; i < 8; i++) {
                motor.backward(0.21f);
                motor.update();
            }
            delay(500);

            //stop for pause
            for(int i = 0; i<3; i++){
                motor.stop();
                motor.update();
            }
            delay(200);
            
            //Turn wheels opposite direction and move forward
            servo_dir.setAngle(turnLeft ?60:120);
            delay(100);
            for (int i = 0; i < 8; i++) {
                motor.forward(0.17f);
                motor.update();
            }
            delay(500);

            //Stop break
            for(int i = 0; i<3; i++){
                motor.stop();
                motor.update();
            }
            delay(200);
        }
        
        //reached target so set wheels stright to face target
        servo_dir.setAngle(90);
        for(int i = 0; i<3; i++){
            motor.stop();
            motor.update();
        }
        delay(200);
        
    }
}