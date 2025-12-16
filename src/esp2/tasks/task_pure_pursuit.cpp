#include "tasks/task_pure_pursuit.h"
#include "esp2/global_state.h"
#include "esp2/config.h"
#include "pure_pursuit.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include "common/data_types.h"
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

void TaskPurePursuit(void *pvParameters) {
    // Wait until the first path is received
    while (!newPathArrived) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (;;) {
        // 1. Path Update Logic
            bool hasNewPath = false;
            PathMessage localPath;

            if (xSemaphoreTake(pathMutex, portMAX_DELAY)) {
                if (newPathArrived) {
                    firstPathReceived = true;
                    hasNewPath = true;
                    localPath = receivedPath;
                    newPathArrived = false;
                }
                xSemaphoreGive(pathMutex);
            }

            if (hasNewPath) {
                purePursuit.set_path(localPath);
                
                if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
                    finishedPath = false;
                    xSemaphoreGive(stateMutex);
                }
            }

        // 2. Emergency Stop Check
        bool isEmergency = false;
        if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
            isEmergency = emergencyStop;
            xSemaphoreGive(stateMutex);
        }

        if (isEmergency) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 3. Compute Control Command
        Pose2D currentPose;
        Velocity velStruct;

        if (xSemaphoreTake(poseMutex, portMAX_DELAY)) {
            currentPose = {posX, posY, yaw};
            velStruct = {velocity, 0.0f};
            xSemaphoreGive(poseMutex);
        }

        MotionCommand cmd = purePursuit.compute_command(currentPose, velStruct);

        // 4. Apply Control Command
        servo_dir.setAngle(cmd.delta_target);

        /*
        * If you decide you use the pid option in the files located in the archive folder
        * uncomment this line
        */
        // pid.setTargetVelocity(cmd.v_target);

        if ((cmd.v_target == 0)){
            motor.stop();
            motor.update();
            finishedPath = true;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
