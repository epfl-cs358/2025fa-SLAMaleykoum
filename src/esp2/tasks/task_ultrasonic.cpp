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

        bool performingTurn = false;

        if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
            performingTurn = isPerformingCreneau;
            xSemaphoreGive(stateMutex);
        }

        // Only trigger emergency if not performing recovery turn
        if (!performingTurn && (dist > 0 ) && (dist < Config::EMERGENCY_DISTANCE)) {

            if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
                emergencyStop = true;
                xSemaphoreGive(stateMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
