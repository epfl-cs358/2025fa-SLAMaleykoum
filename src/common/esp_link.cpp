#include "../../include/common/esp_link.h"
#include <string.h>

void Esp_link::begin(uint32_t baud) {
  ser_.begin(baud, SERIAL_8N1, rx_pin_, tx_pin_);
}

bool Esp_link::sendPose(const Pose2D& p) {
  return sendRaw(MSG_POSE, reinterpret_cast<const uint8_t*>(&p), sizeof(Pose2D));
}

bool Esp_link::sendCorrection(const LoopClosureCorrection& c) {
  return sendRaw(MSG_CORR, reinterpret_cast<const uint8_t*>(&c), sizeof(LoopClosureCorrection));
}

bool Esp_link::sendPath(const GlobalPathMessage& gpm) {
  // n'envoyer que la partie utile du tableau
  const uint16_t n = gpm.current_length <= MAX_PATH_LENGTH ? gpm.current_length : MAX_PATH_LENGTH;

  // payload = [current_length(2)][path_id(4)][timestamp(4)][n * Waypoint]
  const uint16_t way_bytes = n * sizeof(Waypoint);
  const uint16_t payload_len = 2 + 4 + 4 + way_bytes;

  uint8_t buf[2 + 4 + 4 + MAX_PATH_LENGTH*sizeof(Waypoint)];
  uint8_t* w = buf;

  // pack little-endian simple (ESP32 est little-endian)
  *w++ = (uint8_t)(n & 0xFF);
  *w++ = (uint8_t)(n >> 8);

  memcpy(w, &gpm.path_id, 4);      w += 4;
  memcpy(w, &gpm.timestamp_ms, 4); w += 4;

  memcpy(w, gpm.path, way_bytes);  // seulement n waypoints
  return sendRaw(MSG_PATH, buf, payload_len);
}

bool Esp_link::sendPing() { return sendRaw(MSG_PING, nullptr, 60); }
bool Esp_link::sendPong() { return sendRaw(MSG_PONG, nullptr, 60); }

bool sendRaw(uint8_t msg_id, const uint8_t* data, uint16_t len) {
  char final_msg[len];

  switch (msg_id) {
    case MSG_PING: 
      snprintf(final_msg, sizeof(final_msg), "PING from ESP1");
      break;
    case MSG_PONG: 
      snprintf(final_msg, sizeof(final_msg), "PONG from ESP2");
      break;
    default: snprintf(final_msg, sizeof(final_msg), "");
  }
  
  Serial1.println(final_msg);

  return true;
}