#include "common/esp_link.h"
#include "tasks/task_receive_path.h"
#include "esp2/global_state.h"
#include "common/data_types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


void TaskReceivePath(void *pvParameters) {

    PathMessage gpm;
    for (;;) {
        esp_link.poll();
        if (esp_link.get_path(gpm)) {
            if (xSemaphoreTake(pathMutex, portMAX_DELAY)) {
                receivedPath = gpm;
                newPathArrived = true;
                xSemaphoreGive(pathMutex);
            }
            
            if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
                if (!firstPathReceived) {
                    firstPathReceived = true;
                }
                xSemaphoreGive(stateMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}