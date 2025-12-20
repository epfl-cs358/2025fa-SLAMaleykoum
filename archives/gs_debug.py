import socket
import struct
import pygame
import numpy as np
import math
import time
import sys
import csv
from datetime import datetime

# =============================================================================
# CONFIGURATION
# =============================================================================
ESP_IP = "192.168.4.1" 
PORT = 9000
GRID_W, GRID_H = 70, 70
RESOLUTION = 0.2
WINDOW_SCALE = 15
LOG_FILENAME = f"telemetry_trace_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================
def world_to_screen(wx, wy):
    try:
        if not (math.isfinite(wx) and math.isfinite(wy)): return None
        cx, cy = (GRID_W / 2) * WINDOW_SCALE, (GRID_H / 2) * WINDOW_SCALE
        ppm = (1.0 / RESOLUTION) * WINDOW_SCALE
        return int(cx + (wx * ppm)), int(cy - (wy * ppm))
    except: return None

def rle_decode(rle_data, total_size):
    decoded = np.zeros(total_size, dtype=np.int8)
    idx, rle_idx = 0, 0
    while rle_idx < len(rle_data) - 1 and idx < total_size:
        count = int(rle_data[rle_idx]) 
        val   = np.int8(rle_data[rle_idx+1]) 
        write_len = min(count, total_size - idx)
        decoded[idx : idx + write_len] = val
        idx += write_len; rle_idx += 2 
    return decoded

# =============================================================================
# TIMELINE VISUALIZER CLASS
# =============================================================================
TASK_COLORS = {
    1: (180, 0, 255),   # LIDAR (Purple)
    2: (0, 0, 255),     # SYNC (Blue)
    3: (255, 165, 0),   # MAP (Orange)
    4: (0, 255, 255),   # IPC (Cyan)
    5: (255, 255, 0),   # GPLAN (Yellow)
    6: (255, 0, 0),     # MPLAN (Red)
    7: (128, 128, 128), # TCP (Grey)
    8: (0, 255, 0)      # VALIDATOR (Green)
}

TASK_NAMES = {
    0: "IDLE", 1: "LIDAR", 2: "SYNC", 3: "MAP",
    4: "IPC", 5: "GPLAN", 6: "MPLAN", 7: "TCP", 8: "VALID"
}

class Timeline:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)
        self.history = [] # List of dicts
        self.active_tasks = {} # Key: task_id, Value: start_time
        self.max_history_us = 1000000 # Show last 1 second of data
        self.font = pygame.font.SysFont("Arial", 12)
        
    def add_events(self, events):
        # events: list of (timestamp_us, task_id, type, core_id)
        for ts, tid, typ, core in events:
            if typ == 1: # START
                self.active_tasks[tid] = ts
            elif typ == 0: # END
                if tid in self.active_tasks:
                    start = self.active_tasks.pop(tid)
                    duration = ts - start
                    if 0 < duration < 5000000: # Filter outliers
                        self.history.append({'start': start, 'end': ts, 'core': core, 'id': tid})

        # Prune old history
        if self.history:
            # Sort by end time to ensure correct timeline behavior
            self.history.sort(key=lambda x: x['end'])
            latest = self.history[-1]['end']
            limit = latest - self.max_history_us
            self.history = [e for e in self.history if e['end'] > limit]

    def draw(self, surface):
        # Background
        pygame.draw.rect(surface, (20, 20, 20), self.rect)
        pygame.draw.rect(surface, (255, 255, 255), self.rect, 2) # Border

        # Core Separator Line
        mid_y = self.rect.centery
        pygame.draw.line(surface, (100,100,100), (self.rect.left, mid_y), (self.rect.right, mid_y))
        
        # Core Labels
        surface.blit(self.font.render("CORE 0 (Comms/Plan)", True, (200,200,200)), (self.rect.left + 5, self.rect.top + 5))
        surface.blit(self.font.render("CORE 1 (Sensors)", True, (200,200,200)), (self.rect.left + 5, mid_y + 5))

        if not self.history: return

        latest_time = self.history[-1]['end']
        # Scale: pixels per microsecond
        scale = self.rect.width / float(self.max_history_us)
        
        for e in self.history:
            # Calculate position relative to the right edge (latest time)
            # x = right_edge - (time_diff * scale)
            time_ago = latest_time - e['end']
            duration = e['end'] - e['start']
            
            width = max(2, duration * scale)
            x_end = self.rect.right - (time_ago * scale)
            x_start = x_end - width
            
            # Skip if off-screen left
            if x_end < self.rect.left: continue

            # Determine Y position based on Core
            row_height = (self.rect.height / 2) - 10
            y = self.rect.top + 20 if e['core'] == 0 else mid_y + 20
            
            col = TASK_COLORS.get(e['id'], (255,255,255))
            
            # Draw Task Block
            task_rect = pygame.Rect(x_start, y, width, row_height)
            
            # Clip to timeline view
            draw_rect = task_rect.clip(self.rect)
            if draw_rect.width > 0:
                pygame.draw.rect(surface, col, draw_rect)

# =============================================================================
# MAIN LOOP
# =============================================================================
def main():
    print(f"Logging to: {LOG_FILENAME}")
    csv_file = open(LOG_FILENAME, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp_PC", "ESP_Time_us", "Task_ID", "Core", "Event_Type"])

    pygame.init()
    # Increase window height to accommodate the timeline
    total_h = GRID_H * WINDOW_SCALE + 200
    screen = pygame.display.set_mode((GRID_W * WINDOW_SCALE, total_h))
    pygame.display.set_caption("ESP32 Trace Visualizer")
    
    font = pygame.font.SysFont("Consolas", 14)
    
    # Create Timeline Object
    # Placed at the bottom of the screen
    timeline = Timeline(0, GRID_H * WINDOW_SCALE, GRID_W * WINDOW_SCALE, 200)

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(3.0) 
    try: 
        print(f"Connecting to {ESP_IP}:{PORT}...")
        client.connect((ESP_IP, PORT))
        print("Connected!")
    except: 
        print("Connection Failed")
        return

    rx_buffer = b''
    HEADER_SIZE = 96
    
    # State variables
    map_surface = None
    path_points = []
    car_pose = {'x':0, 'y':0, 'theta':0}
    goal_pose = {'x':0, 'y':0, 'theta':0}
    
    last_packet_time = time.time()
    
    # Stats for HUD
    stats = {
        'lidar': 0, 'map': 0, 'fails': 0, 'status': 0, 
        't_gplan': 0, 'heap': 0, 'rx_ms': 0
    }

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        try:
            chunk = client.recv(65536)
            if not chunk: break
            rx_buffer += chunk

            while len(rx_buffer) >= HEADER_SIZE:
                # Find Sync Byte
                header_idx = rx_buffer.find(b'\xBE\xEF')
                if header_idx == -1: rx_buffer = rx_buffer[-1:]; break
                rx_buffer = rx_buffer[header_idx:]
                if len(rx_buffer) < HEADER_SIZE: break
                
                # 1. READ FIXED HEADER
                map_sz = struct.unpack('<I', rx_buffer[2:6])[0]
                g_len  = struct.unpack('<H', rx_buffer[6:8])[0]
                
                # We need to peek at the trace count to know full packet size
                # Header is 96 bytes. Trace count is immediately after (2 bytes).
                if len(rx_buffer) < HEADER_SIZE + 2: break
                
                trace_count = struct.unpack('<H', rx_buffer[96:98])[0]
                trace_bytes = trace_count * 8 # 8 bytes per event
                
                total_sz = HEADER_SIZE + 2 + (g_len * 8) + map_sz + trace_bytes
                
                if len(rx_buffer) < total_sz: break 
                
                # We have a full packet!
                packet = rx_buffer[:total_sz]
                rx_buffer = rx_buffer[total_sz:] 
                
                # --- PROCESSING ---
                now = time.time()
                stats['rx_ms'] = (now - last_packet_time) * 1000.0
                last_packet_time = now
                
                # Decode Standard Data
                cx, cy, ct = struct.unpack('<fff', packet[8:20])
                gx, gy, gt = struct.unpack('<fff', packet[20:32])
                car_pose = {'x':cx, 'y':cy, 'theta':ct}
                goal_pose = {'x':gx, 'y':gy, 'theta':gt}
                
                # Decode Health (Offset 33, 64 bytes)
                try:
                    fmt = '<IIIIIIIIIIIIIIII' 
                    h_data = struct.unpack(fmt, packet[33 : 33 + 64])
                    stats['heap'] = h_data[0]
                    stats['t_gplan'] = h_data[3]
                    stats['lidar'] = h_data[10]
                    stats['map'] = h_data[11]
                    stats['fails'] = h_data[12]
                    stats['status'] = h_data[13]
                except: pass

                # Decode Trace Events
                # Offset = Header(96) + TraceCnt(2) + Path + Map
                trace_offset = HEADER_SIZE + 2 + (g_len * 8) + map_sz
                raw_events = packet[trace_offset : trace_offset + trace_bytes]
                
                parsed_events = []
                for i in range(trace_count):
                    eb = raw_events[i*8 : (i+1)*8]
                    # Struct: uint32 ts, uint8 id, uint8 type, uint8 core, uint8 pad
                    ts, tid, typ, core, _ = struct.unpack('<IBBBB', eb)
                    parsed_events.append((ts, tid, typ, core))
                    csv_writer.writerow([now, ts, tid, core, typ])
                
                timeline.add_events(parsed_events)
                if trace_count > 0:
                        print(f"Received {trace_count} events, latest timestamp: {parsed_events[-1][0]}")
                
                # Decode Path
                path_offset = HEADER_SIZE + 2
                path_payload = packet[path_offset : path_offset + (g_len * 8)]
                path_points = []
                for i in range(g_len):
                    px, py = struct.unpack('<ff', path_payload[i*8 : (i+1)*8])
                    path_points.append((px, py))

                # Decode Map
                if map_sz > 0:
                    map_offset = path_offset + (g_len * 8)
                    rle = np.frombuffer(packet[map_offset : map_offset + map_sz], dtype=np.uint8)
                    try:
                        grid = rle_decode(rle, GRID_W*GRID_H).reshape((GRID_H, GRID_W))
                        rgb = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
                        rgb[:,:] = (100,100,100) # Unknown
                        rgb[grid > 5] = (0,0,0)   # Wall
                        rgb[grid < -5] = (255,255,255) # Free
                        surf = pygame.surfarray.make_surface(np.transpose(rgb, (1,0,2)))
                        map_surface = pygame.transform.scale(surf, (GRID_W*WINDOW_SCALE, GRID_H*WINDOW_SCALE))
                    except: pass

            # --- RENDERING (Inside Loop for sync) ---
            screen.fill((30, 30, 30))
            
            # 1. Map
            if map_surface: screen.blit(map_surface, (0,0))
            
            # 2. Path & Robot
            for p in path_points:
                pp = world_to_screen(p[0], p[1])
                if pp: pygame.draw.circle(screen, (255,200,0), pp, 3)
            
            cp = world_to_screen(car_pose['x'], car_pose['y'])
            if cp: 
                pygame.draw.circle(screen, (255,50,50), cp, 8)
                ex = cp[0] - 15 * math.sin(car_pose['theta'])
                ey = cp[1] - 15 * math.cos(car_pose['theta'])
                pygame.draw.line(screen, (255,50,50), cp, (ex, ey), 3)

            gp = world_to_screen(goal_pose['x'], goal_pose['y'])
            if gp: pygame.draw.circle(screen, (50,255,50), gp, 6)

            # 3. Timeline
            timeline.draw(screen)

            # 4. Legend
            x_leg = 10
            y_leg = GRID_H * WINDOW_SCALE + 10
            for tid, name in TASK_NAMES.items():
                if tid == 0: continue
                col = TASK_COLORS.get(tid, (255,255,255))
                pygame.draw.rect(screen, col, (x_leg, y_leg, 15, 15))
                screen.blit(font.render(name, True, (200,200,200)), (x_leg + 20, y_leg))
                x_leg += 70
            
            # 5. Stats
            info_str = f"Lidar:{stats['lidar']} | Map:{stats['map']} | GPlan:{stats['t_gplan']}us | Heap:{stats['heap']}"
            screen.blit(font.render(info_str, True, (255,255,255)), (10, 10))

            pygame.display.flip()

        except socket.timeout: pass
        except Exception as e: print(e); break
    
    csv_file.close()
    pygame.quit()

if __name__ == "__main__": main()

