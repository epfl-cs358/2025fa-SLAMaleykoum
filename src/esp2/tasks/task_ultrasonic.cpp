#include "tasks/task_ultrasonic.h"
#include "global_state.h"
#include "config.h"
#include "hardware/UltraSonicSensor.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"

void TaskUltrasonic(void *pvParameters) {
    for (;;) {
        /*if (!startSignalReceived) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }*/
        float dist = ultrasonic.readDistance();
        if (dist > 0 && dist < Config::EMERGENCY_DISTANCE && !isPerformingCreneau) {
            motor.stop();
            motor.update();
            emergencyStop = true;

            //say cren needs activation

            
            isCrenFinished = false;
            
            
        } 
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
