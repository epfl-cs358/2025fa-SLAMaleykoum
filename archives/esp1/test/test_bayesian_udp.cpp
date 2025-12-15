#include <WiFi.h>
#include <WiFiUdp.h>
#include "test_common_esp1.h"

WiFiUDP udpautre;
IPAddress broadcastIPautre;
const int UDP_PORT = 9000;
float lastAngleESPautre = 0.0;


const size_t MAP_W = 150;
const size_t MAP_H = 150;
const size_t MAP_SIZE = MAP_W * MAP_H;

uint8_t map_data[MAP_SIZE];

const size_t PAYLOAD_SIZE = 1024;  // safe UDP size
const size_t HEADER_SIZE = 8;      // frame_id(4) + packet_id(2) + packet_count(2)
const size_t PACKET_SIZE = HEADER_SIZE + PAYLOAD_SIZE;

uint32_t frame_id = 0;

void send_map_udp()
{
    size_t packet_count = (MAP_SIZE + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE;

    for (size_t i = 0; i < packet_count; i++)
    {
        udpautre.beginPacket(broadcastIPautre, UDP_PORT);

        // Header
        udpautre.write((uint8_t*)&frame_id, 4);
        uint16_t id = i;
        uint16_t total = packet_count;
        udpautre.write((uint8_t*)&id, 2);
        udpautre.write((uint8_t*)&total, 2);

        // Payload slice
        size_t offset = i * PAYLOAD_SIZE;
        size_t chunk = min(PAYLOAD_SIZE, MAP_SIZE - offset);

        udpautre.write(map_data + offset, chunk);

        udpautre.endPacket();
    }

    frame_id++;
}

void setup_test_bayesian_udp() {
    lidar.start();
    delay(1000);

    // Connect to Wifi in STA mode
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("esp32-lidar-slamaleykoum");
    WiFi.begin(ssid, password);
    delay(3000);

    // Calculate broadcast address
    IPAddress localIP = WiFi.localIP();
    IPAddress subnet = WiFi.subnetMask();
    broadcastIPautre = IPAddress(
        localIP[0] | (~subnet[0]),
        localIP[1] | (~subnet[1]),
        localIP[2] | (~subnet[2]),
        localIP[3] | (~subnet[3])
    );
    
    // Start UDP
    udpautre.begin(UDP_PORT);
    delay(1000);
}

void loop_test_bayesian_udp() {

    lidar.build_scan(&scan, scanComplete, lastAngleESPautre);

    // Send full scan only when scanComplete is true
    if (scanComplete) {
        scanComplete = false;

        // Build packet once
        String packet;
        packet.reserve(scan.count * 12); // approximate size per point

        for (int i = 0; i < scan.count; i++) {
            packet += String(scan.angles[i], 2) + "," +
                    String(scan.distances[i]) + "\n";
        }

        // Send via UDP broadcast
        udpautre.beginPacket(broadcastIPautre, UDP_PORT);
        udpautre.write((uint8_t*)packet.c_str(), packet.length());
        udpautre.endPacket();

        scan.count = 0;
    }

    // Fill map with test pattern for now
    for (int i = 0; i < MAP_SIZE; i++)
        map_data[i] = (uint8_t)(frame_id + i);

    send_map_udp();
    delay(50);  // 20Hz stream (adjust if needed)
}
