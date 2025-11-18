#pragma once
#include <Arduino.h>
#include "data_types.h"

const uint8_t MSG_ID_SIZE = 3; // can encode up to 7 types of messages (2^3 - 1)
const uint8_t BYTE_SIZE = 8;
const int MSG_ID_MASK = 0x00011111;
static constexpr size_t QUEUE_CAP = 8;
static constexpr size_t MAX_TXT_LEN = 255;

struct TxtMsg {
    uint8_t len;
    char     data[MAX_TXT_LEN];
};

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
    Esp_link(HardwareSerial& ser) : ser_(ser) {}

    /**
     * @brief Configure UART
     * Initialise the serial.
     * 
     * @param baud : the baudrate for the communication
     */
    void begin();

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
    bool sendPos(const Pose2D& p);

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
    bool sendText(const char* txt);

    bool get_txt(char* out);
    bool get_pos(Pose2D& out);
    bool get_path(GlobalPathMessage& out);

    private:
    HardwareSerial& ser_;
    const uint8_t RX_ESPS = 13;
    const uint8_t TX_ESPS = 12;
    const uint32_t ESPS_BAUDRATE = 2000000;

    TxtMsg queue_txt[QUEUE_CAP];
    size_t head_txt = 0;
    size_t tail_txt = 0;
    size_t count_txt = 0;

    Pose2D queue_pos[QUEUE_CAP];
    size_t head_pos = 0;
    size_t tail_pos = 0;
    size_t count_pos = 0;

    GlobalPathMessage gpm;

    void push_txt(const char* txt);
    void push_pos(const Pose2D& p);

    /**
     * @brief Formates a message and sends it over UART
     * 
     * @param msg_id the id of the message
     * @param data the data to encode depending on the msg_id
     * @param len the number of bytes of the message
     * 
     * @note used by the more precise functions (sendPose(), ...)
     */
    bool sendRaw(uint8_t msg_id, const uint8_t* data, uint16_t len);

};
