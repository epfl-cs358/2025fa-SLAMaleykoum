/**
 * @file Esp_link.cpp
 * @brief Implementation of the UART communication protocol between two ESP32-S3 boards.
 */
#include "../../include/common/esp_link.h"
#include <string.h>

static constexpr uint8_t MSG_ID_SIZE = 3; // can encode up to 7 types of messages (2^3 - 1)
static constexpr uint8_t BYTE_SIZE = 8;

void Esp_link::begin() {
  ser_.begin(ESPS_BAUDRATE, SERIAL_8N1, RX_ESPS, TX_ESPS);
}

/**
 * @note UART behavior:
 *   - available() returns the number of bytes currently stored in the RX buffer.
 *   - read() reads and returns a single byte (-1 if none available).
 *   - readBytes(buffer, len) reads exactly len bytes, blocking until all are received.
 */
void Esp_link::poll() {
  if (ser_.available() < 1) return;

  uint8_t header = ser_.read();
  uint8_t msg_id = header >> (BYTE_SIZE - MSG_ID_SIZE); 
  uint8_t aux = header & 0b00011111;

  switch (msg_id) {
    case MSG_POSE: {
      if (ser_.available() < sizeof(Pose2D)) return;
      Pose2D p;
      ser_.readBytes(reinterpret_cast<uint8_t*>(&p), sizeof(Pose2D));

      push_pos(p);
      break;
    }
    case MSG_PATH: {
      int count = aux;
      if (ser_.available() <  2 * sizeof(uint32_t)) return;

      ser_.readBytes(reinterpret_cast<uint8_t*>(&gpm_.path_id), sizeof(uint32_t));
      ser_.readBytes(reinterpret_cast<uint8_t*>(&gpm_.timestamp_ms), sizeof(uint32_t));

      if (ser_.available() < count * sizeof(Waypoint)) return;
      ser_.readBytes(reinterpret_cast<uint8_t*>(gpm_.path), count * sizeof(Waypoint));
      gpm_.current_length = count;
      
      gpm_available = true;
      break;
    }
    default: return;
  }
}

void Esp_link::sendPos(const Pose2D& p) {
  ser_.write(MSG_POSE << (BYTE_SIZE - MSG_ID_SIZE));
  ser_.write(reinterpret_cast<const uint8_t*>(&p), sizeof(Pose2D));
}

void Esp_link::sendPath(const GlobalPathMessage& gpm) {
  uint8_t header = MSG_PATH << (BYTE_SIZE - MSG_ID_SIZE);
  header |= gpm.current_length;
  ser_.write(&header, 1);
  ser_.write(reinterpret_cast<const uint8_t*>(&gpm.path_id), sizeof(gpm.path_id));
  ser_.write(reinterpret_cast<const uint8_t*>(&gpm.timestamp_ms), sizeof(gpm.timestamp_ms));
  ser_.write(reinterpret_cast<const uint8_t*>(gpm.path), gpm.current_length * sizeof(Waypoint));
}

bool Esp_link::get_pos(Pose2D& out){
  if (count_pos == 0) return false;
  
  out = queue_pos[head_pos];

  head_pos = (head_pos + 1) % QUEUE_CAP;
  count_pos--;

  return true;
}

bool Esp_link::get_path(GlobalPathMessage& out){
  if (!gpm_available) return false;

  out = gpm_;
  gpm_available = false;
  return true;
}

void Esp_link::push_pos(const Pose2D& p) {
  if (count_pos >= QUEUE_CAP) {
      head_pos = (head_pos + 1) % QUEUE_CAP;
      count_pos--;
  }
  
  Pose2D& slot = queue_pos[tail_pos];
  slot.theta = p.theta;
  slot.timestamp_ms = p.timestamp_ms;
  slot.x = p.x;
  slot.y = p.y;

  tail_pos = (tail_pos + 1) % QUEUE_CAP;
  count_pos++;
}
