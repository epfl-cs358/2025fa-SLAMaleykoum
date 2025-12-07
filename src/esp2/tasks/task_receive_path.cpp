#include "tasks/task_receive_path.h"
#include "esp2/global_state.h"
#include "common/esp_link.h"
#include "common/data_types.h"

void TaskReceivePath(void *pvParameters) {

    for (;;) {
        esp_link.poll();
        GlobalPathMessage gpm;
        if (esp_link.get_path(gpm)) {
            receivedPath = gpm;
            newPathArrived = true; 

            if (!firstPathReceived) {
                firstPathReceived = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}