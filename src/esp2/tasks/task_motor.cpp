/*#include "esp2/tasks/task_motor.h"
#include "esp2/global_state.h"
#include "esp2/config.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"

void TaskMotor(void *pvParameters) {
    for (;;) {
        if (emergencyStop || finishedPath || !startSignalReceived) {
            motor.stop();
            servo_dir.setAngle(90);
        } else {
            motor.forward(Config::MOTOR_POWER_DRIVE);
        }
        motor.update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }       
}
*/
