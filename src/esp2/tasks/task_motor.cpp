#include "esp2/tasks/task_motor.h"
#include "esp2/global_state.h"
#include "esp2/config.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include "esp2/control/recovery_maneuvers.h"



void TaskMotor(void *pvParameters) {

    //Our state machine

    enum class MotorState {
        IDLE,
        DRIVING,
        EMERGENCY_STOPPED,
    };

    MotorState state = MotorState::IDLE;
    bool waitingForNewPath = false;


    for (;;) {
        switch (state) {
            case MotorState::IDLE:
                motor.stop();
                servo_dir.setAngle(90);
                if (firstPathReceived && !emergencyStop) {
                    state = MotorState::DRIVING;
                }
                break;
                
            case MotorState::DRIVING:
                if (finishedPath) {
                    motor.stop();
                    servo_dir.setAngle(90);
                    // TODO: esp_link.sendStatus(MessageType::MISSION_COMPLETE);
                    state = MotorState::IDLE;
                    break;
                }
                
                if (emergencyStop && !isPerformingCreneau) {
                    motor.stop();
                    servo_dir.setAngle(90);
                    // TODO: esp_link.sendStatus(MessageType::OBSTACLE_DETECTED);
                    waitingForNewPath = true;
                    state = MotorState::EMERGENCY_STOPPED;
                    break;
                }
                
                motor.forward(Config::MOTOR_POWER_DRIVE);
                break;
                
            case MotorState::EMERGENCY_STOPPED:
                motor.stop();
                
                if (waitingForNewPath && newPathArrived) {
                    waitingForNewPath = false;
                    
                    
                    if (receivedPath.current_length > 0) {//if we've received a path with more than one waypoint
                        
                        //TODO: this target yaw is wrong, reocmpute everyhting after testing again with IMU to make sure you get correct values when car is turning left or right
                        float targetYaw = Control::calculateTargetYaw(
                            currentPose.x, currentPose.y,
                            receivedPath.path[0].x, //TODO: check if we turn toward the first point or the goal, or what? mayeb  not goal, end up facing wall
                            //TODO : will gloabl planner never give you a poitn facing a wall, norally, it should be far enough from a wall
                            receivedPath.path[0].y
                        );
                        
                        // TODO: esp_link.sendStatus(MessageType::TURNING_IN_PROGRESS);
                        delay(500);
                        
                        isPerformingCreneau = true;
                        Control::performMultiPointTurn(targetYaw);
                        isPerformingCreneau = false;
                        
                        // TODO: esp_link.sendStatus(MessageType::READY_FOR_UPDATES);
                        
                        emergencyStop = false;
                        isCrenFinished = true;
                        newPathArrived = false;
                        state = MotorState::DRIVING;
                    }
                }
                break;
        }
        
        motor.update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
