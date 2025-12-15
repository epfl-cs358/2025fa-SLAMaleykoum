#include "tasks/task_ultrasonic.h"
#include "global_state.h"
#include "config.h"
#include "hardware/UltraSonicSensor.h"
#include "hardware/MotorManager.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

void TaskUltrasonic(void *pvParameters) {
    for (;;) {
        float dist = ultrasonic.readDistance();
        
        if (dist > 0 && dist < Config::EMERGENCY_DISTANCE) {
            bool shouldStop = false;
            
            // Read isPerformingCreneau safely
            if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
                if (!isPerformingCreneau) {
                    shouldStop = true;
                }
                xSemaphoreGive(stateMutex);
            }
            
            if (shouldStop) {
                motor.stop();
                motor.update();
                
                // Write emergencyStop safely
                if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
                    emergencyStop = true;
                    xSemaphoreGive(stateMutex);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}