# ESP-to-ESP High-Speed UART Link  
**Binary communication protocol between two ESP32-S3 boards**

This module implements a fast, compact and deterministic communication layer between two ESP32-S3 boards using a dedicated **2 Mbaud UART link**.  
It transmits structured data (poses, paths…) as raw binary without serialization overhead.

---

## Hardware setup

Two ESP32-S3 DevKitC boards wired as:

| ESP1 pin | ESP2 pin |
|----------|----------|
| TX (GPIO 12) → | RX (GPIO 13) |
| RX (GPIO 13) ← | TX (GPIO 12) |

Notes:
- **GND must be shared**
- Boards may be powered independently (USB/battery)

---

## Features

- **Binary protocol** with minimal overhead  
- **Header-based message identification**  
- **Raw struct transmission** (no JSON, no strings)  
- **High throughput**: 2'000'000 baud  
- **Zero-copy send** using reinterpret_cast  
- **FIFO queue** for Pose2D  
- **Last-value store** for GlobalPathMessage  

---

## Message Types

### Pose2D (`MSG_POSE`)

```cpp
struct Pose2D {
    float x;
    float y;
    float theta;
    uint32_t timestamp_ms;
};
```

Handled with:
- `sendPos()`
- `get_pos()`

Stored in a FIFO queue (capacity 4).

---

### GlobalPathMessage (`MSG_PATH`)

```cpp
struct Waypoint { float x; float y; };

struct GlobalPathMessage {
    Waypoint path[MAX_PATH_LENGTH];
    uint16_t current_length;
    uint32_t path_id;
    uint32_t timestamp_ms;
};
```

Handled with:
- `sendPath()`
- `get_path()`

Only the **most recently received** path is stored.

---

## Protocol Description

Each message begins with a **1-byte header**:

```
bits: [MSG_ID (3 bits)] [unused (5 bits)]
```

Payload follows immediately in raw memory format.

No checksum or framing → assumes clean UART.

---

## API Overview

```cpp
Esp_link link(Serial1);

link.begin();         // initialize UART
link.poll();          // process incoming messages

link.sendPos(p);      // send Pose2D
link.sendPath(gpm);   // send GlobalPathMessage

Pose2D p;
if (link.get_pos(p)) {
    // consume pose
}

GlobalPathMessage path;
link.get_path(path);  // always returns the latest path
```

---

## Internal Behavior

### Pose2D queue
- Circular buffer (capacity 4)
- Overflows drop the oldest entry
- Non-blocking polling

### GlobalPathMessage
- Only the most recent message is kept
