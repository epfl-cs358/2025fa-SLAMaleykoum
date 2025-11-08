#include <Arduino.h>
#include "test_common_esp1.h"
#include "esp1/hardware/rpLidar.h"
#include "esp1/hardware/rpLidarTypes.h"
#include "Config.h"

const char* mqtt_topic_lidar_real = "slamaleykoum77/lidar";

void printLidarAsciiMap(int count) {
    Serial.println("🟢 Printing Lidar ASCII map...");
    const int width = 40;
    const int height = 20;
    const float scale = 100.0f; // 1 character = 10 cm

    char map[height][width];
    memset(map, ' ', sizeof(map));

    for (int i = 0; i < count; ++i) {
        float angle_rad = lidar->Data[i].angle * DEG_TO_RAD;
        float distance = lidar->Data[i].distance;

        if (distance <= 0 || distance > 2000.0f) continue;

        float x = cos(angle_rad) * distance / scale;
        float y = sin(angle_rad) * distance / scale;

        int cx = width / 2 + (int)x;
        int cy = height / 2 - (int)y;

        if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
            map[cy][cx] = '*';
        }
    }

    map[height / 2][width / 2] = 'R'; // Robot center

    Serial.println("🧭 Lidar 2D ASCII map:");
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Serial.print(map[y][x]);
        }
        Serial.println();
    }
}

void setup_test_read_lidar() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("🚀 Initialisation LIDAR + MQTT...");

    connection.setupWifi();
    connection.check_connection();

    // --- Initialisation UART2 pour le lidar ---
    //Serial2.begin(LIDAR_BAUDRATE, SERIAL_8N1, RXD2, TXD2);
    Serial2.setRxBufferSize(LIDAR_SERIAL_BUFFER_SIZE);
    
    // --- Reset et health check ---
    lidar->resetDevice();
    stDeviceStatus_t sdst = lidar->getDeviceHealth();
    printf("📊 Lidar Health: high=%d low=%d status=%d\n",
           sdst.errorCode_high, sdst.errorCode_low, sdst.status);

    // --- Angle d’intérêt complet ---
    lidar->setAngleOfInterest(LIDAR_ANGLE_OF_INTEREST_START, LIDAR_ANGLE_OF_INTEREST_END);

    bool ret = lidar->start(standard);
    char msg[6000];
    if (ret) {
        strcpy(msg, "🟢 RPLidar started correctly!\r\n");
        Serial.println(msg);
    } else {
        strcpy(msg, "🔴 Error starting RPLidar!\r\n");
        Serial.println(msg);
    }
    connection.publish(mqtt_topic_lidar_real, msg);
}

// Boucle principale
void loop_test_read_lidar() {
    connection.check_connection();

    int count = lidar->readMeasurePoints();

    if (count <= 0) {
        Serial.println("⚠️ No lidar data, restarting...");
        lidar->resetDevice();
        lidar->start(standard);
        delay(500);
        return;
    }

    count = min(count, MAX_LIDAR_POINTS);
    char msg[6000];
    strcpy(msg, "{\"type\":\"lidar\",\"points\":[");

    int valid = 0;
    for (int i = 0; i < count; i++) {
        float angle = (lidar->DataBuffer[i].angle_high * 128 + lidar->DataBuffer[i].angle_low / 2) / 64.0f;
        float distance = (lidar->DataBuffer[i].distance_high * 256 + lidar->DataBuffer[i].distance_low) / 4.0f;
        int quality = lidar->DataBuffer[i].quality / 4;

        if (distance == 0 || distance > MAX_RANGE) continue;

        if (valid > 0) strcat(msg, ",");
        char point[64];
        snprintf(point, sizeof(point),
                 "{\"angle\":%.1f,\"distance\":%.1f,\"quality\":%d}",
                 angle, distance / 1000.0f, quality); // mètre ou mm selon besoin
        strcat(msg, point);
        valid++;

        if (valid >= 180) break; // limite MQTT
    }

    strcat(msg, "]}");
    connection.publish(mqtt_topic_lidar_real, msg);

    Serial.printf("📡 Published %d lidar points\n", valid);

    delay(500); // ~2 Hz
}