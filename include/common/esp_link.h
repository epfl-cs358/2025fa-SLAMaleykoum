#pragma once
#include <Arduino.h>
#include "data_types.h"

/**
 * @class Esp_link 
 * @brief all the communication protocol between the 2 esps.
 */
class Esp_link {
    public:
    /**
     * @brief Constructor for the communication.
     * Takes the arguments and save these values
     * 
     * @param ser : the serial on which the communication will happen
     * @param rx_pin, tx_pin : the pins for the communication
     */
    Esp_link(HardwareSerial ser, int rx_pin, int tx_pin) 
        : ser_(ser), rx_pin_(rx_pin), tx_pin_(tx_pin) {}

    /**
     * @brief Configure UART
     * Initialise the serial.
     * 
     * @param baud : the baudrate for the communication
     */
    void begin(uint32_t baud);

    /**
     * @brief collect the messages sent over the communication and keeps them in a buffer
     * 
     * @note call this function in loop
     */
    void poll();

    /**
     * @brief To send a position
     * Serialize a Pose2D and sends it over UART with its ID MSG_POSE
     * 
     * @param p the position to be sent
     */
    bool sendPose(const Pose2D& p);

    /**
     * @brief To send the corrected position
     * Serialize a LoopClosureCorrection and sends it over UART with its ID MSG_CORR
     * 
     * @param c the right position
     */
    bool sendCorrection(const LoopClosureCorrection& c);

    /**
     * @brief To send the path
     * Serialize a GlobalPathMessage and sends it over UART with its ID MSG_PATH
     * 
     * @param gpm the path to send
     */
    bool sendPath(const GlobalPathMessage& gpm);
    
    // just for the tests
    bool sendPing();
    bool sendPong();

    private:
    HardwareSerial ser_;
    int8_t rx_pin_;
    int8_t tx_pin_;

    /**
     * @brief Formates a message and sends it over UART
     * 
     * @param msg_id the id of the message
     * @param data the data to encode depending on the msg_id
     * @param len the length of the message to encode
     * 
     * @note used by the more precise functions (sendPose(), ...)
     */
    bool sendRaw(uint8_t msg_id, const uint8_t* data, uint16_t len);

};
