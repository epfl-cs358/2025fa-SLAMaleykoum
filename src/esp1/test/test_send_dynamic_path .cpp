/**
 * @file main_esp1_dynamic_path_test.cpp
 * @brief ESP1 dynamic path update test.
 *
 * Sends:
 *   - Path A on startup
 *   - Path B after 20 seconds
 *
 * Purpose:
 *   Simulates real-case "obstacle detected → new path generated".
 */

#include <Arduino.h>
#include "common/esp_link.h"
#include "common/data_types.h"

// UART link (TX=12, RX=13, 2Mbaud)
Esp_link esp_link(Serial1);

char buf[100];

// ------------------------------------------
// PATH A (the original one)
// ------------------------------------------
float pathA_X[] = {
    0.00, 0.00, 0.50, 1.00, 1.50, 2.00,
    2.50, 3.00, 3.50, 4.00, 4.00, 4.00,
    4.00, 4.00, 4.00, 3.50, 3.00, 2.50,
    2.00, 1.50, 1.00, 0.50, 0.00, -0.20
};
float pathA_Y[] = {
    0.00, 0.20, 0.20, 0.20, 0.20, 0.20,
    0.20, 0.20, 0.20, 0.20, 0.50, 1.00,
    1.50, 2.00, 2.40, 2.40, 2.40, 2.40,
    2.40, 2.40, 2.40, 2.40, 2.40, 2.40
};
const int pathA_size = sizeof(pathA_X) / sizeof(pathA_X[0]);

// ------------------------------------------
// PATH B (a NEW path for dynamic update)
// ------------------------------------------
float pathB_X[] = {
    4.00, 4.50, 5.00, 5.00, 5.00, 5.00, 5.00
};
float pathB_Y[] = {
    0.20, 0.20, 0.20, 1.70, 2.0, 2.50, 3.00
};
const int pathB_size = sizeof(pathB_X) / sizeof(pathB_X[0]);


// ===============================================================
// SETUP
// ===============================================================
void setup_send_dynamic_path() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== ESP1 Dynamic Path Test ===");

    esp_link.begin();
    Serial.println("UART initialized.");

    // --- Send PATH A -----------------------------------------
    GlobalPathMessage pathA;
    pathA.current_length = pathA_size;
    pathA.path_id = 1;
    pathA.timestamp_ms = millis();

    for (int i = 0; i < pathA_size; i++) {
        pathA.path[i].x = pathA_X[i];
        pathA.path[i].y = pathA_Y[i];
    }

    Serial.println("Sending Path A (initial path)...");
    esp_link.sendPath(pathA);
    Serial.println("Path A sent!");

    // Wait 20 seconds
    Serial.println("\nWaiting 20 seconds before sending new path...");
    delay(10000);

    // --- Send PATH B -----------------------------------------
    GlobalPathMessage pathB;
    pathB.current_length = pathB_size;
    pathB.path_id = 2;  // NEW PATH ID!
    pathB.timestamp_ms = millis();

    for (int i = 0; i < pathB_size; i++) {
        pathB.path[i].x = pathB_X[i];
        pathB.path[i].y = pathB_Y[i];
    }

    Serial.println("Sending Path B (updated path)...");
    esp_link.sendPath(pathB);
    Serial.println("Path B sent!");

    Serial.println("\nDynamic path update test completed.");
}

void loop_send_dynamic_path() {
    // nothing needed
}
