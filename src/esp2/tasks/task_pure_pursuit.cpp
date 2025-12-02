#include "tasks/task_pure_pursuit.h"
#include "esp2/global_state.h"
#include "esp2/config.h"
#include "pure_pursuit.h"
#include "hardware/MotorManager.h"
#include "hardware/DMS15.h"
#include "common/data_types.h"
#include <WiFi.h>

void TaskPurePursuit(void *pvParameters) {
    while (!newPathArrived) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (;;) {
        // 1. Path Update Logic
        if (newPathArrived) {
            purePursuit.set_path(receivedPath);
            finishedPath = false;
            
            if (tcpClient && tcpClient.connected()) {
                tcpClient.printf("PATH_START: ID=%d LEN=%d\n", receivedPath.path_id, receivedPath.current_length);
                for(int i=0; i<receivedPath.current_length; i++) {
                    tcpClient.printf("WP: %d %.3f %.3f\n", i, receivedPath.path[i].x, receivedPath.path[i].y);
                    vTaskDelay(2);
                }
                tcpClient.println("PATH_END");
            }
            newPathArrived = false;
        }

        // 2. TCP Connection
        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                Serial.printf("Client connected from %s\n", tcpClient.remoteIP().toString().c_str());
            }
        }

        // 3. Read Commands
        if (tcpClient && tcpClient.connected()) {
            while (tcpClient.available()) {
                String line = tcpClient.readStringUntil('\n');
                line.trim();
                if (line == "start") {
                    startSignalReceived = true;
                    Serial.println("🦄 Start command received! 🦄");
                }
            }
        }

        if (!startSignalReceived) {
            motor.stop();
            motor.update();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        Pose2D currentPose = {posX, posY, yaw};
        Velocity velStruct = {velocity, 0.0f};

        MotionCommand cmd = purePursuit.compute_command(currentPose, velStruct);
        
        Waypoint lh = purePursuit.get_lookahead_point();
        float Kp_val = purePursuit.get_kp();
        float Ld_val = purePursuit.get_ld();

        // 4. Send Telemetry
        if (tcpClient && tcpClient.connected()) {
            char tcpBuf[256];
            int len = snprintf(tcpBuf, sizeof(tcpBuf),
                               "CMD: v=%.3f d=%.3f | P: Kp=%.2f Ld=%.2f | T: LX=%.3f LY=%.3f | POSE: X=%.2f Y=%.2f Yaw=%.2f Vel=%.3f\n",
                               cmd.v_target, cmd.delta_target,
                               Kp_val, Ld_val,
                               lh.x, lh.y,
                               posX, posY, yaw, velocity);
            tcpClient.write((const uint8_t*)tcpBuf, len);
        }

        servo_dir.setAngle(cmd.delta_target);

        if ((cmd.v_target == 0)){
            motor.stop();
            motor.update();
            finishedPath = true;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
