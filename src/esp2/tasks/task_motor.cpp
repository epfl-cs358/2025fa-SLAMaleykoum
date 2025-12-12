#include "esp2/tasks/task_motor.h"
#include "esp2/global_state.h"
#include "esp2/config.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"

void TaskMotor(void *pvParameters) {
    for (;;) {
        if (emergencyStop || finishedPath || !firstPathReceived /*|| !startSignalReceived*/){
            motor.stop();
            servo_dir.setAngle(90);
            
            if(emergencyStop && !isCrenFinished){
                isPerformingCreneau = true;
                delay(1000);
                for(int i = 0; i<10; i++){
                    motor.backward(0.21f);
                    motor.update();
                }
                delay(800);
                for(int i = 0; i<3; i++){
                    motor.stop();
                    motor.update();
                }

                for(int i = 0 ; i< 5; i++){
            
                
                    servo_dir.setAngle(120);
                    delay(500);
                    for(int i = 0; i<8; i++){
                        motor.backward(0.21f);
                        motor.update();
                    }

                    delay(800);

                    for(int i = 0; i<3; i++){
                        motor.stop();
                        motor.update();
                    }

                    delay(1500);

                    servo_dir.setAngle(60);
                    delay(500);
                    for(int i = 0; i<8; i++){
                        motor.forward(0.17f);
                        motor.update();
                    }

                    delay(800);

                    for(int i = 0; i<3; i++){
                        motor.stop();
                        motor.update();
                    }

                    delay(1500);
                }

                servo_dir.setAngle(90);

                isCrenFinished = true;
                // CLEAR FLAG: Re-enable emergency stop detection
                isPerformingCreneau = false;
                
                // Clear emergency stop so vehicle can continue
                emergencyStop = false;
            }
        } else {
            motor.forward(Config::MOTOR_POWER_DRIVE);
        }
        motor.update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}