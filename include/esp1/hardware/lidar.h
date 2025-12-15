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
	 * Starts the Lidar and its measurement system
	 * 
	 * @return true if mode started correctly, false if not 
	 */	
	bool start();

    /**
	 * should be excecuted as often as possible to read the data from USART
	 * 
	 * @return the number of data for the running mode (express *40, standard *1)
	 */	
	uint16_t readMeasurePoints();

    /**
	 * Calculates angle for Standard mode 
	 * According to the Datasheet for standard mode angle´s
	 * 
	 * @param LS
	 * @param MS
	 * 
	 * @returns angle 0.00-360.00
	 */	
	float calcAngle(uint8_t _lowByte, uint8_t _highByte);

    /**
	 * Calculates the distance for Standard mode
	 * According to the Datasheet for standard mode distance´s
	 * 
	 * @returns distance in mm
	 */	
	float calcDistance(uint8_t _lowByte, uint8_t _highByte);

	/**
	 * Builds a full scan from the Lidar
	 * 
	 * @param pointer to LiDARScan struct to fill
	 * @param reference to bool that becomes true if a full scan is complete
	 * @param reference to last angle read by ESP (to detect full rotation)
	 */
	void build_scan(LiDARScan* scan, bool &scanComplete_, float& lastAngleESP_);

	/**
	 * Build a scan in real time
	 * While decoding the serial, we build the scan
	 * 
	 * @param scan to LiDARScan struc to fill
	 * @param scancomplete the bool that becomes true if a full scan is complete
	 * @param lastAngle the last angle read by ESP (to detect full rotation)
	 * 
	 * @return number of points in the scan
	 */
	uint16_t readScanLive(LiDARScan* scan, bool &scanComplete, float &lastAngle);

private:
	static constexpr uint32_t LIDAR_SERIAL_BUFFER_SIZE = 5000;
    static constexpr uint8_t LIDAR_RX_PIN = 5;
    static constexpr uint8_t LIDAR_TX_PIN = 4;
    static constexpr uint32_t LIDAR_BAUDRATE = 460800;
	static constexpr uint8_t DATA_SIZE = 5;

    /**
	 * Tries to read a new full cycle of Points
	 * 
	 * @return the number of points 
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
