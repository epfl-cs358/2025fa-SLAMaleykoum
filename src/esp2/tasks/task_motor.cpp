#include "esp2/tasks/task_motor.h"
#include "esp2/global_state.h"
#include "esp2/config.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include "esp2/control/recovery_maneuvers.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

void TaskMotor(void *pvParameters) {
    bool recoveryTriggered = false;

    for (;;) {
        bool localEmergency = false;
        bool localFinished = false;
        bool localFirstPath = false;

        if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
            localEmergency = emergencyStop;
            localFinished = finishedPath;
            localFirstPath = firstPathReceived;
            xSemaphoreGive(stateMutex);
        }

        // EMergency stop trigger so recovery maneuver
        if (localEmergency && !recoveryTriggered) {
            //recoveryTriggered = true;

            motor.stop();
            servo_dir.setAngle(Config::STRAIGHT_ANGLE);

            isPerformingCreneau = true;
            Control::performMultiPointTurn();
            isPerformingCreneau = false;

            if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
                emergencyStop = false;
                isCrenFinished = true;
                xSemaphoreGive(stateMutex);
            }
        }

        // Reset local emergency to false after recover maneuver done
        if (!localEmergency) {
            recoveryTriggered = false;
        }

        // Continue driving 
        if (!localEmergency && !localFinished && localFirstPath) {
            motor.forward(Config::MOTOR_POWER_DRIVE);
        } else {
            motor.stop();
            servo_dir.setAngle(Config::STRAIGHT_ANGLE);
        }

        motor.update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

  