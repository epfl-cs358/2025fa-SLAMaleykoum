/**
 * @file Esp_link.h
 * @brief UART-based communication module between two ESP32-S3 boards.
 *
 * This file defines the Esp_link class, which implements a lightweight
 * binary protocol for exchanging structured data over a dedicated high-speed UART link. 
 * The class handles message framing, raw struct transmission, polling of incoming bytes, 
 * and small buffers for retrieved messages.
 *
 * Only binary data is transmitted; no serialization or dynamic allocation is used.
 */
#pragma once
#include <Arduino.h>
#include "data_types.h"

/**
 * @class Esp_link
 * @brief Minimal UART protocol between two ESP32 boards.
 *
 * Handles binary message transmission and reception over a dedicated high-speed UART link. 
 * Provides non-blocking polling, message decoding, and small internal buffers for received data.
 *
 * Workflow:
 *  - begin() must be called once to configure the UART.
 *  - poll() processes incoming bytes. It must be called often (in loop() for ex.)
 */
class Esp_link {
    public:
    /**
     * @brief Constructs an Esp_link object bound to a specific HardwareSerial port.
     * Stores a reference to the UART instance that will be used for all inter-ESP communication.
     *
     * @param ser Reference to the HardwareSerial interface to use.
     */
    Esp_link(HardwareSerial& ser) : ser_(ser) {}

    /**
     * @brief Initializes the UART used for ESP-to-ESP communication.
     *
     * Sets up the hardware serial port `ser_` with the configured baud rate,
     * 8N1 frame format, and the designated RX/TX pins. Must be called before
     * any send or receive operation.
     */
    void begin();

    /**
     * @brief Polls the UART and processes any incoming ESP-to-ESP message.
     *
     * Reads the message header, identifies the message type, and retrieves the
     * corresponding payload. Payloads are pushed into the appropriate internal queues.
     *
     * The function returns immediately if not enough bytes are available to
     * fully decode the current message.
     *
     * @note UART behavior:
     *   - available() returns the number of bytes currently stored in the RX buffer.
     *   - read() reads and returns a single byte (-1 if none available).
     *   - readBytes(buffer, len) reads exactly len bytes, blocking until all are received.
     */
    void poll();

    /**
     * @brief Sends a Pose2D message over the ESP-to-ESP UART link.
     *
     * Writes the MSG_POSE header byte and then transmits the Pose2D structure
     * as raw binary data. The payload is sent exactly as it is laid out in memory,
     * allowing fast and compact communication.
     *
     * @param p Pose2D struct to transmit.
     */
    void sendPos(const Pose2D& p);

    /**
     * @brief Sends a PathMessage message over the ESP-to-ESP UART link.
     *
     * Writes the MSG_PATH header byte and then transmits the PathMessage structure
     * as raw binary data. The payload is sent exactly as it is laid out in memory,
     * allowing fast and compact communication.
     * 
     * gpm.current_length waypoints are sent even if some of them are not initialised. 
     *
     * @param gpm PathMessage struct to transmit.
     */
    void sendPath(const PathMessage& pm);

    /**
     * @brief Retrieves the oldest pending Pose2D message from the queue.
     *
     * If at least one Pose2D is available, copies it into @p out, advances
     * the queue head, and returns true. Returns false if the queue is empty.
     *
     * @param out Reference where the retrieved Pose2D will be stored.
     * @return true if a Pose2D was retrieved, false if the queue is empty.
     */
    bool get_pos(Pose2D& out);

    /**
     * @brief Retrieves the most recently received PathMessage.
     *
     * Copies the internally stored PathMessage into @p out and
     * returns true. This function does not perform queueing: only the
     * latest received path is kept.
     *
     * @param out Reference where the stored PathMessage will be written.
     * @return true always, since a path is always considered available.
     */
    bool get_path(PathMessage& out);

    private:
    // static constexpr size_t QUEUE_CAP = 1;
    static constexpr uint8_t RX_ESPS = 13;
    static constexpr uint8_t TX_ESPS = 12;
    static constexpr uint32_t ESPS_BAUDRATE = 2000000;

    HardwareSerial& ser_;

    Pose2D pos_;
    bool pos_available = false;

    PathMessage gpm_;
    bool gpm_available = false;
};
