/** @filename: esp1/hardware/lidar.h
 *  @description: Contract for the LiDAR sensor driver and
 *               feature extraction pipeline on ESP_1.
 * 
 *  @note: the raw sensor data must be simplified and refined
 *       before it can be used by the sophisticated mathematical
 *       models like the EKF SLAM or the Bayesian Occupancy Grid.
 * 
 *  @job: Run the raw data through Feature Extraction to create a
 *        simplified version of the scan.
 */

#pragma once

#include "common/data_types.h"
#include <cstdint>

// List of Response descriptors
extern rp_descriptor_t resp_descriptor[];

// List of Request Messages
extern rq_message_t req_message[];

/**
 * @brief Driver and processing class for the LiDAR unit.
 * Manages communication with the LiDAR and performs feature extraction.
 */
class Lidar {
public:
    /**
	 * Construcor of Class
	 *
	 * @param ser the Serial to use USART and collect data
	 */
	Lidar(HardwareSerial& ser) : serial(ser) {}

	// Storage to save the Data of a scan
    rawScanDataPoint_t DataBuffer[3250];

    /**
	 * Initializes the LiDAR device and starts standard measurement mode.
	 *
	 * This function configures the serial interface, resets the LiDAR device, clears
	 * the serial buffer, and sends a scan request command. It waits for the LiDAR
	 * descriptor response and verifies that the device correctly entered scan mode.
	 *
	 * Steps performed internally:
	 * 1. Set serial RX buffer size.
	 * 2. Begin serial communication with LiDAR (baud rate, pins, 8N1 format).
	 * 3. Send reset command and wait 1 second.
	 * 4. Clear any remaining bytes in the serial buffer.
	 * 5. Send standard scan request command to LiDAR.
	 * 6. Wait for descriptor response with a timeout of 5 seconds.
	 * 7. Compare received descriptor with expected startScan descriptor.
	 *
	 * @return true if LiDAR entered scanning mode correctly, false if a timeout occurred
	 *         or the response descriptor did not match.
	 */	
	bool start();

    /**
	 * Reads measurement points from the LiDAR in the currently active mode.
	 *
	 * This function should be called as often as possible to continuously fetch LiDAR
	 * data. It internally calls awaitStandardScan() to collect points for standard scans.
	 * The number of points is the count of points in a full rotation.
	 *
	 * @return Number of measurement points read from the LiDAR.
	 */	
	uint16_t readMeasurePoints();

    /**
	 * Calculates angle
	 * According to the Datasheet for standard mode angle´s
	 * 
	 * @param LS
	 * @param MS
	 * 
	 * @returns angle in degrees 0.00-360.00
	 */	
	float calcAngle(uint8_t _lowByte, uint8_t _highByte);

    /**
	 * Calculates the distance
	 * According to the Datasheet for standard mode distance´s
	 * 
	 * @returns distance in mm
	 */	
	float calcDistance(uint8_t _lowByte, uint8_t _highByte);

	/**
	 * Builds a full LiDAR scan in a user-provided LiDARScan struct.
	 *
	 * This function reads raw points from the LiDAR using readMeasurePoints() and
	 * converts them to human-readable distances (mm), angles(degrees), and quality values.
	 * The scan is considered complete when the LiDAR completes a full rotation
	 * (angle wraps around from high to low).
	 *
	 * @param scan Pointer to LiDARScan struct to fill with converted measurement data.
	 * @param scanComplete_ Reference to a boolean indicating whether a full scan is complete.
	 *                      This function updates it to true when a full rotation is detected.
	 * @param lastAngleESP_ Reference to the last angle read by the ESP, used to detect full rotation.
	 */
	void build_scan(LiDARScan* scan, bool &scanComplete_, float& lastAngleESP_);

private:
	static constexpr uint32_t LIDAR_SERIAL_BUFFER_SIZE = 5000;
    static constexpr uint8_t LIDAR_RX_PIN = 5;
    static constexpr uint8_t LIDAR_TX_PIN = 4;
    static constexpr uint32_t LIDAR_BAUDRATE = 460800;

    /**
	 * Attempts to read a complete standard scan cycle (full rotation) from the LiDAR device.
	 *
	 * This function continuously reads raw measurement points from the serial interface
	 * and stores them into the internal DataBuffer. 
	 * It detects the start of a LiDAR frame using the quality and angle bits of each point. 
	 * Once a full rotation is detected, it returns the total number of points collected. 
	 * If no full rotation is completed within 5 seconds, the function returns the number 
	 * of points read so far.
	 *
	 * @return Number of valid measurement points stored in DataBuffer.
	 */	
	uint16_t awaitStandardScan();

	/**
	 * Checks if no Serial Data is available in a given time
	 * 
	 * @param wait time ms 
	 * @param amount of expected bytes
	 * 
	 * @return true if a timeout happend
	 */	
	bool checkForTimeout(uint32_t _time,size_t _size);

    /**
	 * Compares two Response Descriptors 
	 * 
	 * @returns true if equal 
	 */	
	bool compareDescriptor(rp_descriptor_t _descr1,rp_descriptor_t _descr2);

    // pointer to HardwareSerial USART 
    HardwareSerial& serial;
};
