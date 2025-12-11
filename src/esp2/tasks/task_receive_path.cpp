#include "common/esp_link.h"
#include "tasks/task_receive_path.h"
#include "esp2/global_state.h"
#include "common/data_types.h"

#include "common/wifi_connection.h"

// char msgReceive[100];
// const char* mqtt_topic_connection_esp2_receive= "slamaleykoum77/setupesp2";

void TaskReceivePath(void *pvParameters) {

    PathMessage gpm;
    // connection.setupWifi();
    // connection.check_connection();
    // connection.publish(mqtt_topic_connection_esp2_receive, "Wifi Setup success");
    for (;;) {
        esp_link.poll();
        if (esp_link.get_path(gpm, GLOBAL)) {
            receivedPath = gpm;
            newPathArrived = true; 
            // snprintf(msgReceive, sizeof(msgReceive), "coucou j'ai recu");
            
            // connection.publish(mqtt_topic_connection_esp2_receive, msgReceive);

            if (!firstPathReceived) {
                firstPathReceived = true;
            }
            
            // for (uint16_t i = 0; i < gpm.current_length; i++) {
            //     connection.check_connection();
            //     snprintf(msgReceive, sizeof(msgReceive), 
            //                 "Waypoint %d: x = %.2f, y = %.2f", 
            //                 i, gpm.path[i].x, gpm.path[i].y);

            //     connection.publish(mqtt_topic_connection_esp2_receive, msgReceive);
            // }
        }
        // snprintf(msgReceive, sizeof(msgReceive), "coucou j'ai pas recu, NUUUUUUUL");
            
        // connection.publish(mqtt_topic_connection_esp2_receive, msgReceive);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}