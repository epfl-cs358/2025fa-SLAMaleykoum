/*#include "tasks/task_pure_pursuit.h"
#include "esp2/global_state.h"
#include "esp2/config.h"
#include "pure_pursuit.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include "common/data_types.h"
#include <WiFi.h>

void TaskPurePursuit(void *pvParameters) {
    // Wait until the first path is received
    while (!newPathArrived) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (;;) {
        // 1. Path Update Logic
        if (newPathArrived) {
            purePursuit.set_path(receivedPath);
            finishedPath = false;
            newPathArrived = false;
        }

        // 2. Emergency Stop Check
        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 3. Compute Control Command
        Pose2D currentPose = {posX, posY, yaw};
        Velocity velStruct = {velocity, 0.0f};

        MotionCommand cmd = purePursuit.compute_command(currentPose, velStruct);

        // 4. Apply Control Command
        servo_dir.setAngle(cmd.delta_target);
        // pid.setTargetVelocity(cmd.v_target);

        if ((cmd.v_target == 0)){
            motor.stop();
            motor.update();
            finishedPath = true;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
*/
