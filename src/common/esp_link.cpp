#include "../../include/common/esp_link.h"
#include <string.h>

void Esp_link::begin(uint32_t baud) {
  ser_.begin(baud, SERIAL_8N1, rx_pin_, tx_pin_);
}

bool Esp_link::sendPose(const Pose2D& p) {
  return sendRaw(MSG_POSE, (uint8_t*)(&p), sizeof(Pose2D));
}

bool Esp_link::sendCorrection(const LoopClosureCorrection& c) {
  return sendRaw(MSG_CORR, (uint8_t*)(&c), sizeof(LoopClosureCorrection));
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

/**
 * @note messages de max 2^(8+5)-1 = 8191 characteres
 */
bool Esp_link::sendText(const char* txt) { 
  return sendRaw(MSG_TXT, reinterpret_cast<uint8_t*>(&txt), strlen(txt)); 
}

bool Esp_link::sendRaw(uint8_t msg_id, uint8_t* data, uint16_t len) {
  uint8_t* payload;
  uint8_t header = (msg_id << (BYTE_SIZE - MSG_ID_SIZE));
  uint8_t len_lo;

  switch (msg_id) {
    case MSG_PATH: {
      int count = 0; // ------------
      header |= count;
      break;
    }
    case MSG_TXT: {
      uint8_t len_hi = (len >> 8) & 0x3F;
      len_lo = len & 0xFF;

      uint8_t header = (msg_id << 6) | len_hi;
      
      payload = data;
      break;
    }
    default: return false;
  }

  ser_.write(header);
  if(msg_id == MSG_TXT) ser_.write(len_lo);
  ser_.write((uint8_t*)&payload, sizeof(payload));

  return true;
}

void Esp_link::push_txt(const char* txt) {
  size_t len = strlen(txt);
  if (len > MAX_TXT_LEN) len = MAX_TXT_LEN;

  // if the queue is full, drop the oldest
  if (count_txt >= QUEUE_CAP_TXT) {
      head_txt = (head_txt + 1) % QUEUE_CAP_TXT;
      count_txt--;
  }
  
  TxtMsg& slot = queue_txt[tail_txt];
  slot.len = len;

  memcpy(slot.data, txt, len);
  slot.data[len] = '\0';

  tail_txt = (tail_txt + 1) % QUEUE_CAP_TXT;
  count_txt++;
}

bool Esp_link::get_txt(char* &out) {
  if (count_txt == 0) return false;
  
  TxtMsg msg = queue_txt[head_txt];

  strncpy(out, msg.data, msg.len - 1);  
  out[msg.len - 1] = '\0';

  head_txt = (head_txt + 1) % QUEUE_CAP_TXT;
  count_txt--;

  return true;
}

/**
 * @note: pour le README : 
 *    - available() renvoie le nbr de bytes qu'on peut lire mtn 
 * (= le nbr de byte deja recus et stockés dans le buffer RX)
 *    - read() lit et retourne 1 seul octet (-1 si pas de data)
 *    - readBytes(buffer, length) lit exactement length bytes dans un buffer 
 * en bloquant jusqu'a tt recevoir
 */
void Esp_link::poll() {
  if (ser_.available() < 1) return;

  uint8_t header = ser_.read();
  uint8_t msg_id = header >> (BYTE_SIZE - MSG_ID_SIZE); 
  uint8_t aux    = header & MSG_ID_MASK;

  switch (msg_id) {

    case MSG_POSE: {
        if (ser_.available() < sizeof(Pose2D)) return;
        Pose2D p;
        ser_.readBytes((uint8_t*)&p, sizeof(Pose2D));
        // traiter p
        break;
    }

    case MSG_PATH: {
        uint8_t count = aux;  
        if (ser_.available() < count * sizeof(Waypoint)) return;

        Waypoint buf[64];
        ser_.readBytes((uint8_t*)buf, count*sizeof(Waypoint));
        // traiter buf[0..count-1]
        break;
    }

    case MSG_CORR: break;
    case MSG_TXT: {
      uint8_t len_hi = aux;

      if (ser_.available() < 1) return;
      uint8_t len_lo = ser_.read();

      uint16_t len = (len_hi << 8) | len_lo;

      if (ser_.available() < len) return;

      char buf[len + 1];
      ser_.readBytes((uint8_t*)buf, len);
      buf[len] = '\0';

      push_txt(buf);
      break;
    }

    default: return;
  }
}