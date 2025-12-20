
🏠 [Backlink to the main README](/README.md)

# Ground Station - Real-Time SLAM Visualization

## Overview

Live telemetry dashboard. Displays real-time mapping, navigation, and CPU profiling data over WiFi.

**Features:**
- Live occupancy grid map
- Robot pose and navigation path visualization
- Real-time task execution timeline (CPU profiling)
- Performance metrics and system health monitoring
- Automatic telemetry logging
- Frontier detection overlay (Not currently working)

## Quick Start

### Connect & Run

1. **Connect to car WiFi:**
   ```
   SSID: LIDAR_AP
   Password: l1darpass
   ```

2. **Launch ground station:**
   ```bash
   python ground_station.py
   ```

NOTE: It is normal that the car starts mooving before you've got time to connect. If you can't connect, checkout [the trouble shooting page](/assets/docs/troubleshooting.md).

## Display Layout

```
┌─────────────────────────────────────────────────┐
│  HUD (Top-Left)          MAP WINDOW             │
│  - Robot Position        - Black: Walls         │
│  - Goal Position         - White: Free Space    │
│  - System State          - Grey: Unknown        │
│  - Performance Stats     - Red Dot: Robot       │
│  - Memory Usage          - Green Circle: Goal   │
│                          - Yellow dots: Path    │
│                          - Blue Trail: History  │
│                                                 │
│ (Cyan frontiers, are not working at the moment) │
├─────────────────────────────────────────────────┤
│  TIMELINE (CPU Profiling)                       │
│  ┌─ Core 0 ──────────────────────────────────┐  │
│  │ [IPC] [GPLAN] [MPLAN] [TCP]               │  │
│  └───────────────────────────────────────────┘  │
│  ┌─ Core 1 ──────────────────────────────────┐  │
│  │ [LIDAR] [SYNC] [MAP]                      │  │
│  └───────────────────────────────────────────┘  │
├─────────────────────────────────────────────────┤
│  LEGEND: Task Colors                            │
│  ■ LIDAR  ■ SYNC  ■ MAP  ■ IPC  ■ GPLAN ...     │
└─────────────────────────────────────────────────┘
```

## What You'll See

### Map Visualization
- **Occupancy Grid**: Bayesian map showing explored areas
- **Robot (Red Dot)**: Current position with heading arrow
- **Goal (Green Circle)**: Mission planner target with heading
- **Path (Yellow Dots)**: A* global path from robot to goal
- **Frontiers (Cyan Dots)**: Unexplored boundary cells for autonomous exploration
- **Trail (Blue)**: Historical path showing where robot has been

### Timeline Panel
Real-time CPU profiling showing task execution on both cores:
- **Horizontal bars**: Task execution periods
- **Bar width**: Execution duration
- **Bar color**: Task type (see legend)
- **Two lanes**: Core 0 (top) and Core 1 (bottom)
- **Scrolling window**: Shows last 1 second of activity

Note: More information about the profiling can be found [here](assets/docs/debugging/CPU_PROFILING.md)

### HUD Information

```
POS   : X 2.45 Y 1.23        ← Robot position (meters)
GOAL  : X 3.50 Y 2.10        ← Current navigation goal
STATE : EXPLORATION          ← Mission state
--- PERF (us) ---
MAP UPDATE : 1834            ← Bayesian grid update time
GLOB PLAN  : 4678            ← A* pathfinding time
MISS PLAN  : 1245            ← Mission planning time
--- CORE 1 DIAG ---
LIDAR RX  : 14               ← Total LiDAR scans received
MAP UPDT  : 18               ← Successful map updates
SYNC DROP : 4                ← Dropped sync frames
MTX FAIL  : 2                ← Mutex acquisition failures
--- MEM (KB) ---
FREE HEAP : 856.2 KB         ← Available RAM
MIN HEAP  : 842.8 KB         ← Minimum free RAM seen
--- STACKS ---
TCP:1234 GP:2345 MP:3456 LI:4567  ← Stack watermarks
ESP2:45ms PKT:5678            ← ESP2 connection & packet count
TRACE: 1847 events            ← Profiling events in packet
```

## Data Logging

All telemetry is automatically logged to CSV:

**Filename Format:** `telemetry_trace_YYYYMMDD_HHMMSS.csv`

**Columns:**
```csv
Timestamp_PC,ESP_Time_us,Task_ID,Core,Event_Type
1702350445.123,1234567,3,1,1
1702350445.135,1246789,3,1,0
```

- `Timestamp_PC`: Python timestamp when packet received
- `ESP_Time_us`: ESP32 microsecond timestamp
- `Task_ID`: 1-8 (see task mapping below)
- `Core`: 0 or 1
- `Event_Type`: 1=START, 0=END

Use this CSV with `plot_trace.py` for offline analysis.

## Task ID Mapping

| ID | Name | Core | Function |
|----|------|------|----------|
| 1 | LIDAR | 1 | LiDAR data acquisition |
| 2 | SYNC | 1 | Pose-scan fusion |
| 3 | MAP | 1 | Bayesian grid updates |
| 4 | IPC | 0 | Inter-ESP communication |
| 5 | GPLAN | 0 | Global path planning |
| 6 | MPLAN | 0 | Mission planning |
| 7 | TCP | 0 | Telemetry transmission |
| 8 | VALID | 0 | Pose validation |

## Configuration

Edit constants at the top of `ground_station.py`:

```python
# Network
ESP_IP = "192.168.4.1"
PORT = 9000

# Map (MAKE SURE THEY MATCH WITH main_esp1.cpp and data_typed.h)
GRID_W = 70           # Grid width (cells)
GRID_H = 70           # Grid height (cells)
RESOLUTION = 0.15     # Cell size (meters)
WINDOW_SCALE = 13     # Display scale factor

# Timeline
max_history_us = 1000000  # Show last 1 second (line 169)
```

## How It Works

### 1. TCP Protocol

The ESP32 sends binary packets every 100ms containing:

```
┌─────────────┬──────────┬──────────┬─────────┬─────────┐
│ Header (96B)│ Trace(2B)│ Path(var)│ Map(var)│ Events  │
│             │  Count   │          │  (RLE)  │ (var)   │
└─────────────┴──────────┴──────────┴─────────┴─────────┘
```

**Header Contains:**
- Magic bytes (0xBEEF)
- Map size (uint32)
- Path length (uint16)
- Robot pose (3x float)
- Goal pose (3x float)
- System state (uint8)
- System health (64 bytes)

### 2. Map Decompression

Maps are transmitted using Run-Length Encoding (RLE):

```python
def rle_decode(rle_data, total_size):
    # Input: [Count, Value, Count, Value, ...]
    # Output: Decompressed int8 array (log-odds)
    decoded = np.zeros(total_size, dtype=np.int8)
    for count, value in pairs(rle_data):
        decoded[idx:idx+count] = value
        idx += count
    return decoded
```

Converts log-odds to RGB:
- `> 4`: Occupied → Black
- `< -6`: Free → White
- Otherwise: Unknown → Grey

### 3. Timeline Rendering

```python
class Timeline:
    def add_events(self, events):
        # Match START/END pairs
        for ts, task_id, event_type, core in events:
            if event_type == START:
                active_tasks[task_id] = ts
            elif event_type == END:
                start = active_tasks.pop(task_id)
                duration = ts - start
                history.append({
                    'start': start,
                    'end': ts,
                    'core': core,
                    'id': task_id
                })
```

Draws horizontal bars for each completed task execution.

### 4. Coordinate Transforms

**World → Screen:**
```python
def world_to_screen(wx, wy):
    # Convert meters to pixels
    pixels_per_meter = (1.0 / RESOLUTION) * WINDOW_SCALE
    sx = center_x + (wx * pixels_per_meter)
    sy = center_y - (wy * pixels_per_meter)  # Invert Y
    return int(sx), int(sy)
```

**World → Grid:**
```python
def world_to_index(wx, wy):
    # Convert meters to grid indices
    ix = int(wx / RESOLUTION) + (GRID_W // 2)
    iy = (GRID_H // 2) - int(wy / RESOLUTION)
    return ix, iy
```

## Troubleshooting

### Connection Issues

**Can't connect to robot:**
```
Error: [Errno 111] Connection refused
```
- Check WiFi connection to `LIDAR_AP`
- Verify esp1 is powered on
- Verify you uploaded the correct code _(sadly happens way to often)_

## Keyboard Controls

- **c**: Clear car trace
- **ctrl + w**: close window $\rightarrow$ Save CSV and exit

## Output Files

When you close the ground station:
```
Session complete. Log saved to: telemetry_trace_20241220_153045.csv
```

Use this file with:
```bash
python plot_trace.py telemetry_trace_20241220_153045.csv
```

## Integration with Profiling System

The ground station is part of the complete profiling workflow:

1. **ESP32** → Logs task events with `log_event()`
2. **TCP Task** → Transmits events every 100ms
3. **Ground Station** → Displays live timeline + saves CSV
4. **plot_trace.py** → Generates detailed Gantt charts from CSV

See [`CPU_PROFILING_README.md`](/assets/docs/debugging/CPU_PROFILING.md) for the complete system documentation.

---

**Last Updated**: December 2025