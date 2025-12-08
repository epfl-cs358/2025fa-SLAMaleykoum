import socket
import struct
import pygame
import numpy as np
import math
import time

# --- CONFIGURATION ---
ESP_IP = "192.168.4.1" 
PORT = 9000
GRID_W = 200        
GRID_H = 200        
RESOLUTION = 0.05    
WINDOW_SCALE = 4    

# --- COLORS ---
C_BLACK  = (0, 0, 0)
C_WHITE  = (255, 255, 255)
C_GREY   = (100, 100, 100)
C_RED    = (255, 50, 50)
C_GREEN  = (50, 255, 50)
C_YELLOW = (255, 200, 0)

# --- COORDINATE TRANSFORMS ---
def world_to_screen(wx, wy):
    try:
        if not (math.isfinite(wx) and math.isfinite(wy)): return None
        cx_screen = (GRID_W / 2) * WINDOW_SCALE
        cy_screen = (GRID_H / 2) * WINDOW_SCALE
        pixels_per_meter = (1.0 / RESOLUTION) * WINDOW_SCALE
        sx = cx_screen + (wx * pixels_per_meter)
        sy = cy_screen - (wy * pixels_per_meter) 
        sx = max(-20000, min(20000, sx))
        sy = max(-20000, min(20000, sy))
        return int(sx), int(sy)
    except: return None

# --- RLE DECOMPRESSION ---
def rle_decode(rle_data, total_size):
    """
    Decodes RLE data: [count, val, count, val...]
    Returns a numpy array of size total_size
    """
    decoded = np.zeros(total_size, dtype=np.int8)
    idx = 0
    rle_idx = 0
    
    # We need at least 2 bytes (count, val) to proceed
    while rle_idx < len(rle_data) - 1 and idx < total_size:
        # Force cast to standard python int to avoid numpy uint8 overflow
        count = int(rle_data[rle_idx]) 
        val   = np.int8(rle_data[rle_idx+1]) 
        
        # Safety clamp to prevent writing past the end of the array
        write_len = min(count, total_size - idx)
        
        decoded[idx : idx + write_len] = val
        
        idx += write_len
        rle_idx += 2
        
    return decoded

def main():
    pygame.init()
    screen = pygame.display.set_mode((GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE))
    pygame.display.set_caption("ESP32 Mission Control")
    font = pygame.font.SysFont("Consolas", 16, bold=True)
    
    trail_surface = pygame.Surface((GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE), pygame.SRCALPHA)

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(5.0) 
    print(f"Connecting to {ESP_IP}:{PORT}...")
    try:
        client.connect((ESP_IP, PORT))
        print("Connected.")
    except Exception as e:
        print(f"Connection Failed: {e}")
        return

    rx_buffer = b''
    HEADER_SIZE = 64
    
    car_pose  = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    goal_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    path_points = []  
    history_trail = [] 
    
    map_surface = None

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                history_trail = []
                trail_surface.fill((0,0,0,0)) 

        try:
            chunk = client.recv(65536)
            if not chunk: break
            rx_buffer += chunk

            while True:
                if len(rx_buffer) < HEADER_SIZE: break 

                header_idx = rx_buffer.find(b'\xBE\xEF')
                if header_idx == -1:
                    rx_buffer = rx_buffer[-1:]
                    break
                
                if header_idx > 0:
                    rx_buffer = rx_buffer[header_idx:]
                
                if len(rx_buffer) < HEADER_SIZE: break
                
                # Payload sizes
                map_bytes_size = struct.unpack('<I', rx_buffer[2:6])[0]
                path_len = struct.unpack('<H', rx_buffer[6:8])[0]
                
                total_packet_size = HEADER_SIZE + (path_len * 8) + map_bytes_size
                
                if len(rx_buffer) < total_packet_size: break 
                
                packet = rx_buffer[:total_packet_size]
                rx_buffer = rx_buffer[total_packet_size:]

                # --- 1. DECODE HEADER ---
                cx, cy, ct = struct.unpack('<fff', packet[8:20])
                gx, gy, gt = struct.unpack('<fff', packet[20:32])
                
                car_pose = {'x': cx, 'y': cy, 'theta': ct}
                goal_pose = {'x': gx, 'y': gy, 'theta': gt}

                # --- 2. TRAIL ---
                if not history_trail or math.hypot(cx - history_trail[-1][0], cy - history_trail[-1][1]) > 0.1:
                    history_trail.append((cx, cy))
                    if len(history_trail) > 2000: history_trail.pop(0)
                    pt = world_to_screen(cx, cy)
                    if pt:
                        pygame.draw.circle(trail_surface, (0, 150, 255, 50), pt, 4)

                # --- 3. PATH ---
                path_payload = packet[HEADER_SIZE : HEADER_SIZE + (path_len * 8)]
                path_points = []
                for i in range(path_len):
                    px, py = struct.unpack('<ff', path_payload[i*8 : (i+1)*8])
                    path_points.append((px, py))

                # --- 4. DECODE MAP (UPDATED FOR RLE) ---
                if map_bytes_size > 0:
                    map_payload = packet[HEADER_SIZE + (path_len * 8) :]
                    
                    # FIX: Use int8 for data interpretation, but read as uint8 for RLE logic
                    rle_data = np.frombuffer(map_payload, dtype=np.uint8)
                    
                    try:
                        grid_log_odds = rle_decode(rle_data, GRID_W * GRID_H).reshape((GRID_H, GRID_W))
                        
                        rgb_map = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
                        
                        occ_mask  = grid_log_odds > 8  
                        free_mask = grid_log_odds < -8  
                        unk_mask  = (~free_mask) & (~occ_mask)

                        rgb_map[free_mask] = C_WHITE
                        rgb_map[occ_mask]  = C_BLACK
                        rgb_map[unk_mask]  = C_GREY
                        
                        surf = pygame.surfarray.make_surface(np.transpose(rgb_map, (1, 0, 2)))
                        map_surface = pygame.transform.scale(surf, (GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE))
                    except Exception as e:
                        print(f"RLE Decode Error: {e}")

            # --- RENDER ---
            screen.fill(C_GREY)
            if map_surface: screen.blit(map_surface, (0, 0))
            screen.blit(trail_surface, (0,0))

            pts_screen = [world_to_screen(p[0], p[1]) for p in path_points]
            valid_pts = [p for p in pts_screen if p is not None]
            if len(valid_pts) > 1:
                pygame.draw.lines(screen, C_YELLOW, False, valid_pts, 3)
                for pt in valid_pts: pygame.draw.circle(screen, C_YELLOW, pt, 2)

            gx_s, gy_s = world_to_screen(goal_pose['x'], goal_pose['y'])
            if gx_s:
                pygame.draw.circle(screen, C_GREEN, (gx_s, gy_s), 8, 2) 
                ge_x = gx_s - 20 * math.sin(goal_pose['theta']) 
                ge_y = gy_s - 20 * math.cos(goal_pose['theta'])
                pygame.draw.line(screen, C_GREEN, (gx_s, gy_s), (ge_x, ge_y), 2)

            cx_s, cy_s = world_to_screen(car_pose['x'], car_pose['y'])
            if cx_s:
                pygame.draw.circle(screen, C_RED, (cx_s, cy_s), 7)
                ce_x = cx_s + 20 * (-math.sin(car_pose['theta']))
                ce_y = cy_s - 20 * (math.cos(car_pose['theta']))
                pygame.draw.line(screen, C_RED, (cx_s, cy_s), (ce_x, ce_y), 3)

            status_lines = [
                f"CAR   : X{car_pose['x']:5.2f} Y{car_pose['y']:5.2f} T{math.degrees(car_pose['theta']):5.1f}",
                f"PATH  : {len(path_points)} pts",
                f"MAP B : {map_bytes_size} bytes (Compressed)"
            ]
            for i, line in enumerate(status_lines):
                screen.blit(font.render(line, True, C_BLACK), (12, 12 + i * 20))
                screen.blit(font.render(line, True, C_YELLOW), (10, 10 + i * 20))

            pygame.display.flip()

        except socket.timeout: pass 
        except Exception as e: 
            print(e)
            break

    client.close()
    pygame.quit()

if __name__ == "__main__":
    main()