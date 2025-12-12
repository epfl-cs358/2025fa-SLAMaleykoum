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

extern rp_descriptor_t resp_descriptor[];	///< List of Response descriptors 
extern rq_message_t req_message[];			///< List of Request Messages
extern rq_Packet_t req_Express[];			///< Request Message for Express Scan Modes

/**
 * @brief Driver and processing class for the LiDAR unit.
 * Manages communication with the LiDAR and performs feature extraction.
 */
class Lidar {
public:
    /**
	 * Construcor of Class
	 *
	 * @param pointer to used USART
	 * @param Baudrate
	 */
	Lidar(HardwareSerial& ser) : serial(ser) {}

    /**
	 * Starts the Lidar and its measurement system
	 * 
	 * @param modus to run the lidar
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
	 * Checks if no Serial Data ist available in a given time
	 * 
	 * @param wait time ms 
	 * @param amount of expected bytes
	 * @return true if a timeout happend
	 */	
	bool checkForTimeout(uint32_t _time,size_t _size);

    /**
	 * Compares two Response Descriptors 
	 * @returns true if equal 
	 */	
	bool compareDescriptor(rp_descriptor_t _descr1,rp_descriptor_t _descr2);

    // Storage to save the Data of a Standard Scan
    rawScanDataPoint_t DataBuffer[3250];

    /**
	 * Calculates angle for Standard mode 
	 * According to the Datasheet for standard mode angle´s
	 * @param LS
	 * @param MS
	 * @returns angle 0.00-360.00
	 */	
	float calcAngle(uint8_t _lowByte, uint8_t _highByte);

    /**
	 * Calculates the distance for Standard mode
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
	 */
	uint16_t readScanLive(LiDARScan* scan, bool &scanComplete, float &lastAngle);

private:
	static constexpr uint32_t LIDAR_SERIAL_BUFFER_SIZE = 5000;
    static constexpr uint8_t LIDAR_RX_PIN = 5;
    static constexpr uint8_t LIDAR_TX_PIN = 4;
    static constexpr uint32_t LIDAR_BAUDRATE = 460800;

    /**
	 * Tries to read a new full cycle of Points
	 * 
	 * @return the number of points 
	 */	
	uint16_t awaitStandardScan();

    /**
     * @brief Converts raw data points into high-confidence, distinct features (Landmarks).
     * This is where clustering, corner detection, or other algorithms would live.
     * @param raw_points The raw sensor data.
     * @return A vector of extracted Landmarks for the EKF-SLAM.
     */
    //std::vector<LiDARLandmark> extract_features(const std::vector<RawLiDARPoint>& raw_points);
    // RIEN A FAIRE DANS LA CLASS LIDAR NON??????????????????????????????????????????????????????????????????????

    // pointer to HardwareSerial USART 
    HardwareSerial& serial;


};

