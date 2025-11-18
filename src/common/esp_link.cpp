#include "../../include/common/esp_link.h"
#include <string.h>

void Esp_link::begin() {
  ser_.begin(ESPS_BAUDRATE, SERIAL_8N1, RX_ESPS, TX_ESPS);
}

bool Esp_link::sendPos(const Pose2D& p) {
  return sendRaw(MSG_POSE, reinterpret_cast<const uint8_t*>(&p), sizeof(Pose2D));
}

bool Esp_link::sendCorrection(const LoopClosureCorrection& c) {
  return sendRaw(MSG_CORR, reinterpret_cast<const uint8_t*>(&c), sizeof(LoopClosureCorrection));
}

bool Esp_link::sendPath(const GlobalPathMessage& gpm) {
  return sendRaw(MSG_PATH, reinterpret_cast<const uint8_t*>(&gpm), sizeof(gpm));
}

/**
 * @note messages de max 2^(8+5)-1 = 8191 characteres
 */
bool Esp_link::sendText(const char* txt) { 
  return sendRaw(MSG_TXT, reinterpret_cast<const uint8_t*>(&txt), strlen(txt)); 
}

bool Esp_link::sendRaw(uint8_t msg_id, const uint8_t* data, uint16_t len) {
  uint8_t header = (msg_id << (BYTE_SIZE - MSG_ID_SIZE));
  uint8_t len_lo;

  switch (msg_id) {
    case MSG_PATH:
    case MSG_POSE: break;
    case MSG_TXT: {
      uint8_t len_hi = (len >> 8) & 0x3F;
      len_lo = len & 0xFF;

      uint8_t header = (msg_id << 6) | len_hi;
      break;
    }
    default: return false;
  }

  ser_.write(header);
  if(msg_id == MSG_TXT) ser_.write(len_lo);

  Serial.print("sendRaw bytes: ");
    for (uint16_t i = 0; i < len; ++i) {
        Serial.printf("%02X ", data[i]);
    }
  Serial.println();

  ser_.write(data, len);

  return true;
}

void Esp_link::push_pos(const Pose2D& p) {

  // if the queue is full, drop the oldest
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

void Esp_link::push_txt(const char* txt) {
  size_t len = strlen(txt);
  if (len > MAX_TXT_LEN-1) len = MAX_TXT_LEN-1;

  // if the queue is full, drop the oldest
  if (count_txt >= QUEUE_CAP) {
      head_txt = (head_txt + 1) % QUEUE_CAP;
      count_txt--;
  }
  
  TxtMsg& slot = queue_txt[tail_txt];
  slot.len = len;

  memcpy(slot.data, txt, len);
  slot.data[len] = '\0';

  tail_txt = (tail_txt + 1) % QUEUE_CAP;
  count_txt++;
}

bool Esp_link::get_pos(Pose2D& out){
  if (count_pos == 0) return false;
  
  out = queue_pos[head_pos];

  head_pos = (head_pos + 1) % QUEUE_CAP;
  count_pos--;

  return true;
}

bool Esp_link::get_path(GlobalPathMessage& out){
  out = gpm;
  return true;
}

/**
 * @note give a buffer out that has MAX_TXT_LEN
 */
bool Esp_link::get_txt(char* out) {
  if (count_txt == 0) return false;
  
  TxtMsg& msg = queue_txt[head_txt];

  strncpy(out, msg.data, msg.len);
  out[msg.len] = '\0';

  head_txt = (head_txt + 1) % QUEUE_CAP;
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
        ser_.readBytes(reinterpret_cast<uint8_t*>(&p), sizeof(Pose2D));
        
        Serial.printf("RX Pose: x=%.3f, y=%.3f, th=%.3f, t=%u\n",
                  p.x, p.y, p.theta, p.timestamp_ms);

        push_pos(p);
        break;
    }

    case MSG_PATH: {
        if (ser_.available() < sizeof(GlobalPathMessage)) return;
        ser_.readBytes(reinterpret_cast<uint8_t*>(&gpm), sizeof(GlobalPathMessage));
        break;
    }
    
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