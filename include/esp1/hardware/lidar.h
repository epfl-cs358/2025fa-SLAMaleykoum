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
#include <vector>
#include <cstdint>

// Define maximum points the raw sensor can return for buffer allocation
const size_t MAX_RAW_LIDAR_POINTS = 360; 

/**
 * @brief Represents a single raw point from the LiDAR sensor.
 */
struct RawLiDARPoint {
    float angle_rad; // Angle of the measurement
    float distance_m; // Distance measurement (range)
    uint8_t quality; // Quality/intensity of the return
};

/**
 * @brief Driver and processing class for the LiDAR unit.
 * Manages communication with the LiDAR and performs feature extraction.
 */
class Lidar {
public:
    Lidar();

    /**
     * @brief Initializes the LiDAR hardware (e.g., UART connection, motor control).
     * @return true if initialization was successful.
     */
    bool initialize();

    /**
     * @brief Performs the full read, processing, and feature extraction pipeline.
     * This is the primary method called by the SLAM task.
     * @return A LiDARScan struct containing only the extracted Landmarks (features).
     */
    LiDARScan get_processed_scan();

private:
    /**
     * @brief Low-level function to communicate with the hardware and read a full sweep.
     * @param raw_points Output vector to fill with raw sensor data.
     * @return The timestamp of the completed scan.
     */
    uint32_t read_raw_data(std::vector<RawLiDARPoint>& raw_points);

    /**
     * @brief Converts raw data points into high-confidence, distinct features (Landmarks).
     * This is where clustering, corner detection, or other algorithms would live.
     * @param raw_points The raw sensor data.
     * @return A vector of extracted Landmarks for the EKF-SLAM.
     */
    std::vector<LiDARLandmark> extract_features(const std::vector<RawLiDARPoint>& raw_points);
};

