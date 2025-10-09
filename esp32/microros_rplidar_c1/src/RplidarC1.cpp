/**
 * @file    RplidarC1.cpp
 * @author  2025sp-turboslam (last year's team)
 * @brief   Source file for RPLIDAR C1 control class.
 * @version XX.X
 * @date    XX-XX-2025
 *
 *
 * @details
 *  This module provides public functions for initializing,
 *  comunicating and controlling an RPLIDAR C1 360° laser scanner.
 *  It supports starting, stopping, and reading scan data.
 */

#include "RplidarC1.h"

// Constructor => create an instance of the RplidarC1 (lidar) class
RplidarC1::RplidarC1()
    : dataIndex(0),         // Current index in the data buffer
    pointAlign(false),      // Whether we are point-aligned
    frameActive(false),     // Whether we are frame-aligned
    frameComplete(true) {}  // Whether a full frame has been received



// Initialize serial communication with the RPLIDAR C1
void RplidarC1::begin()
{
    const int rxPin = 5; // rx pin: recive data from the lidar (in our case GPIO5 on the ESP32)
    const int txPin = 4; // tx pin: send commands to the lidar

    Serial2.setRxBufferSize(8000);                          // Avoid overflow for 3d scans.
                                                            // (Serial2 is UART2 on the ESP32)
    //          baud-rate | protocol  | RX pin | TX pin
    Serial2.begin(460800,   SERIAL_8N1, rxPin,   txPin);    // begin serial communication with the lidar
    
    // Initialize LaserScan message
    scan_msg.header.frame_id.data = (char *)"laser_frame";
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    scan_msg.angle_min = MIN_ANGLE;
    scan_msg.angle_max = MAX_ANGLE;
    scan_msg.angle_increment = ANGLE_INCREMENT;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1; // Assuming 10 Hz scan rate
    scan_msg.range_min = MIN_RANGE;
    scan_msg.range_max = MAX_RANGE;
}



// What does it do ? Set the motor speed of the lidar
// (We don't use it, because we set the speed with the "start scan" command
//  which sets the speed to a fixed value depending on the scan rate)
// -----------------------------------------------------------
// void RplidarC1::setMotorPWM(uint16_t pwm)
// {
//     // [0xA5, 0xF0, PWM_L, PWM_H]
//     uint8_t cmd[4];
//     cmd[0] = 0xA5;
//     cmd[1] = 0xF0;
//     cmd[2] = pwm & 0xFF;
//     cmd[3] = pwm >> 8;
//     Serial2.write(cmd, sizeof(cmd));
//     // give the motor controller a moment to adjust
//     delay(50);
//     Serial.printf("Motor PWM set to %u\r\n", pwm);
// }
// -----------------------------------------------------------


/* 
 * Reset lidar: 
 * sends the 2-byte command sequence {0xA5, 0x40} over
 * the UART serial line to the LiDAR sensor.
 * When the LiDAR receives this:
 *  - It stops any ongoing scan or data transmission.
 *  - It reboots its internal firmware (like pressing a reset button).
 *  - The motor typically stops spinning for a second.
 *  - After reboot, it’s ready to accept new commands.
 */
void RplidarC1::resetLidar()
{
    uint8_t resetCommand[] = {0xA5, 0x40};             // 0xA5 is the start byte, 0x40 is the reset command
    Serial2.write(resetCommand, sizeof(resetCommand)); // Send reset command to the lidar
    Serial.println("LIDAR reset command sent");
}


// Start lidar scanning
void RplidarC1::startLidar()
{
    // choose your desired scan-rate here:
    //   ~330 → ~10 Hz, ~500 → ~15 Hz, ~660 → ~20 Hz, etc.
    // setMotorPWM(660);

    // now issue the normal “start scan” command

    uint8_t startCommand[] = {0xA5, 0x20};           // 0xA5 is the start byte, 0x20 is the start scan command
    Serial2.write(startCommand, sizeof(startCommand));
    Serial.println("LIDAR start command sent");
}


// Read data from the lidar until a full frame is received or timeout occurs
// Returns the number of points in the frame, or 0 on timeout/error
int RplidarC1::uartRx()
{
    uint8_t byte;
    unsigned long timeout = millis() + 3000; // 3 second timeout hardcoded ?

    frameComplete = false;  // Reset frame complete flag

    while (millis() < timeout)
    {
        if (!pointAlign)    // we are not point aligned => look for point alignment
                            // Point alignment means that we start reading data from the start of a point
                            // A point is 5 bytes long, and starts with a byte where bits 0 and 4 are set (0x1X and 0xX1)
                            // This ensures that we read complete points
        {
            // while (Serial2.available() > 10)
            //      Serial2.read();
            //  if(Serial2.available() < 5 ){
            //      return 0; //continue;
            //  }
            byte = Serial2.read();
            if ((byte & 0x11) != 0x10) // look for 0x1X (first byte of a point)
                continue;           // we have point alignment
            byte = Serial2.read();
            if ((byte & 0x1) != 0x1) // look for 0xX1 (second byte of a point)
                continue;            // we have point alignment
            for (int i = 0; i < 3; i++)
                byte = Serial2.read(); // read 3rd 4th and 5th byte of this point
            pointAlign = true;
            dataIndex = 0;
            printf("%lu point alignment\r\n", millis());
            continue;
        }
        if (!frameActive)   // we are point aligned but not frame aligned
                            // Frame alignment means that we have detected the start of a new scan frame
                            // A new scan frame starts with a point where bit 0 of the quality byte is set (0x1X)
                            // and bit 1 is clear (0xX0) and bit 0 of the angle_low byte is set (0xX1)

                            // A scan frame is a full 360° rotation of the lidar

                            // The angle is encoded in two bytes:
                            // angle_high byte is the byte that contains the high bits of the angle measurement
                            // angle_low byte is the byte that contains the low bits of the angle measurement
                            // quality byte is the byte that contains the quality of the measurement
                            // For mor information about the angle and distance bytes: see the RPLIDAR C1 datasheet
        {
            if (Serial2.available() < 10) // wait until we have at least 10 bytes (2 points)
                continue;
            Serial2.read(DataBuffer, 5); // read first point (5 bytes)
            // check if this point is the start of a new frame
            auto *point = (stScanDataPoint_t *)DataBuffer; // cast the first 5 bytes to a point structure
            if ((point->quality & 0x01) && !(point->quality & 0x02) && (point->angle_low & 0x01))
            { // we have frame alignment
                frameActive = true; 
                dataIndex = 5;
            } // else we are still looking for frame alignment
            continue;
        }


        // we are point aligned and frameAligned
        // read 5 bytes
        if (Serial2.available() < 10)
            continue;
        Serial2.read(DataBuffer + dataIndex, 5);
        auto *point = (stScanDataPoint_t *)(DataBuffer + dataIndex);
        // check if this point is the start of a new frame
        if ((point->quality & 0x01) && !(point->quality & 0x02) && (point->angle_low & 0x01))
        { // we have a new frame
            if (dataIndex > 2000)
            { // we have a full frame (2000 bytes = 400 points)
                frameComplete = true;
                // printf("%lu frameComplete at dataIndex=%d\r\n",millis(),dataIndex);
            }
            else
            { // frame too small, discard it
                frameActive = false;
                pointAlign = false;
                dataIndex = 5;
                continue;
            }
            // return number of points in the frame
            int count = dataIndex / 5;
            dataIndex = 5;
            return count;
        }

        // continue reading points
        dataIndex += 5;
        if (dataIndex >= BUF_SIZE) // overflow error
        {
            printf("%lu Overflow error at dataIndex=%d\r\n", millis(), dataIndex);
            pointAlign = false;
            frameActive = false;
            dataIndex = 0;
            return 0;
        }
    }

    // in case of timeout
    pointAlign = false;
    frameActive = false;
    dataIndex = 0;
    return 0;
}


// Process a complete frame of data and populate the LaserScan message
// num_points is the number of points in the frame
// Each point is 5 bytes long (quality, angle_low, angle_high, distance_low, distance_high)
void RplidarC1::processFrame(int num_points)
{
    // int num_points = dataIndex / 5; // Each point is 5 bytes

    // For each point in the frame, decode the raw data points from the DataBuffer
    // and populates the ranges and intensities arrays.
    for (int i = 0; i < num_points; i++)
    {
        stScanDataPoint_t *point = (stScanDataPoint_t *)(DataBuffer + i * 5);
        // Distance has distance high and low bytes because it is a 16-bit value
        uint16_t distance = (point->distance_high << 8) | point->distance_low;
        // uint16_t angle = ((point->angle_high << 8) | point->angle_low) >> 1; // 0.01° per unit
        float angle = (point->angle_high * 128 + point->angle_low / 2) / 64.0;

        float posf = angle * (num_points - 1) / 360.0;
        int pos = num_points - 1 - int(posf + 0.5);
        if (pos < 0)
            pos = 0;
        if (pos >= num_points)
            pos = num_points - 1;
        ranges[pos] = ((float)distance) / 4000.0; // Convert to meters (RPLIDAR C1 sends: 4 * distance(in mm)
        intensities[pos] = point->quality / 4;
        // printf("ranges[%d]=%.3f\r\n",pos,ranges[pos]);
    }

    // Populate the LaserScan message
    scan_msg.header.stamp.sec = millis() / 1000;
    scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    scan_msg.ranges.data = ranges;
    scan_msg.ranges.size = num_points;
    scan_msg.intensities.data = intensities;
    scan_msg.intensities.size = num_points;
    scan_msg.angle_increment = 2.0 * PI / num_points;
}