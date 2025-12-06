import socket
import struct
import pygame
import numpy as np
import math
import time

# --- CONFIGURATION ---
ESP_IP = "192.168.4.1" 
PORT = 9000
GRID_W = 200        # Width of the grid (cells)
GRID_H = 200        # Height of the grid (cells)
RESOLUTION = 0.05   # Meters per cell
WINDOW_SCALE = 4    # Pixel scaling (Window size = 800x800)

# --- COLORS (R, G, B) ---
C_BLACK  = (0, 0, 0)
C_WHITE  = (255, 255, 255)
C_GREY   = (100, 100, 100) # Unknown areas
C_RED    = (255, 50, 50)   # The Car
C_GREEN  = (50, 255, 50)   # The Goal
C_BLUE   = (0, 150, 255)   # History Trail
C_YELLOW = (255, 200, 0)   # Planned Path

# --- COORDINATE TRANSFORMS ---
def world_to_screen(wx, wy):
    """ 
    Converts World Meters to Screen Pixels.
    World: X is Right, Y is Up (North).
    Screen: X is Right, Y is Down.
    """
    # Center of screen is (0,0) in World
    cx_screen = (GRID_W / 2) * WINDOW_SCALE
    cy_screen = (GRID_H / 2) * WINDOW_SCALE
    
    pixels_per_meter = (1.0 / RESOLUTION) * WINDOW_SCALE

    sx = cx_screen + (wx * pixels_per_meter)
    sy = cy_screen - (wy * pixels_per_meter) # Invert Y axis
    return int(sx), int(sy)

def main():
    pygame.init()
    screen = pygame.display.set_mode((GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE))
    pygame.display.set_caption("ESP32 Mission Control")
    font = pygame.font.SysFont("Consolas", 16, bold=True)
    
    # --- CONNECTION ---
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(3.0) # 3s timeout
    print(f"Connecting to {ESP_IP}:{PORT}...")
    try:
        client.connect((ESP_IP, PORT))
        print("Connected.")
    except Exception as e:
        print(f"Connection Failed: {e}")
        return

    # --- STATE VARIABLES ---
    rx_buffer = b''
    HEADER_SIZE = 64 # Matches ESP32 struct padding
    
    car_pose  = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    goal_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    path_points = []  # List of (x,y) tuples
    history_trail = [] # List of (x,y) tuples for past positions
    
    map_surface = None

    running = True
    while running:
        # 1. EVENT HANDLING
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            # Add 'C' to clear history if it gets too messy
            if event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                history_trail = []

        # 2. DATA RECEPTION
        try:
            chunk = client.recv(8192)
            if not chunk: 
                print("Server closed connection.")
                break
            rx_buffer += chunk

            # Process buffer until we run out of full packets
            while True:
                # Need at least a header to know payload size
                if len(rx_buffer) < HEADER_SIZE:
                    break 

                # Check Magic Word (0xBE, 0xEF)
                if rx_buffer[0] != 0xBE or rx_buffer[1] != 0xEF:
                    # Sync lost, slide buffer by 1 byte to find magic word
                    rx_buffer = rx_buffer[1:]
                    continue
                
                # Peek at header to calculate total packet size
                # Format: Magic(2), MapSz(4), PathLen(2), Car(12), Goal(12), Padding(32)
                # Map Size is at offset 2 (4 bytes, unsigned int)
                # Path Len is at offset 6 (2 bytes, unsigned short)
                map_bytes_size = struct.unpack('<I', rx_buffer[2:6])[0]
                path_len = struct.unpack('<H', rx_buffer[6:8])[0]
                
                total_packet_size = HEADER_SIZE + (path_len * 8) + map_bytes_size
                
                if len(rx_buffer) < total_packet_size:
                    break # Wait for more data
                
                # --- PACKET EXTRACTED ---
                packet = rx_buffer[:total_packet_size]
                rx_buffer = rx_buffer[total_packet_size:] # Remove from buffer

                # A. Decode Header
                # Car: Offsets 8-20 (3 floats)
                cx, cy, ct = struct.unpack('<fff', packet[8:20])
                # Goal: Offsets 20-32 (3 floats)
                gx, gy, gt = struct.unpack('<fff', packet[20:32])
                
                car_pose = {'x': cx, 'y': cy, 'theta': ct}
                goal_pose = {'x': gx, 'y': gy, 'theta': gt}

                # Update History (Only if moved significantly to save RAM/CPU)
                if not history_trail or math.hypot(cx - history_trail[-1][0], cy - history_trail[-1][1]) > 0.05:
                    history_trail.append((cx, cy))
                    if len(history_trail) > 1000: history_trail.pop(0)

                # B. Decode Path (Array of Waypoints)
                # Payload starts after header
                path_payload = packet[HEADER_SIZE : HEADER_SIZE + (path_len * 8)]
                path_points = []
                for i in range(path_len):
                    px, py = struct.unpack('<ff', path_payload[i*8 : (i+1)*8])
                    path_points.append((px, py))

                # C. Decode Map
                map_payload = packet[HEADER_SIZE + (path_len * 8) :]
                grid = np.frombuffer(map_payload, dtype=np.uint8).reshape((GRID_H, GRID_W))
                
                # Convert Grid to RGB Surface
                # 0=Black, 255=White, 127=Grey
                rgb_map = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
                occ_mask  = grid < 85      # v≈255*(1-prob), donc prob>0.66
                free_mask   = grid > 170     # prob<0.33
                unk_mask   = (~free_mask) & (~occ_mask)

                rgb_map[free_mask] = C_WHITE
                rgb_map[occ_mask]  = C_BLACK
                rgb_map[unk_mask]  = C_GREY
                
                # Pygame surfaces are (Width, Height). Numpy is (Row, Col) -> (Height, Width).
                # We transpose and scale up
                surf = pygame.surfarray.make_surface(np.transpose(rgb_map, (1, 0, 2)))
                map_surface = pygame.transform.scale(surf, (GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE))

            # --- RENDERING ---
            screen.fill(C_GREY)
            
            # 1. Draw Map
            if map_surface:
                screen.blit(map_surface, (0, 0))

            # 2. Draw History Trail (Blue)
            if len(history_trail) > 1:
                pts_screen = [world_to_screen(p[0], p[1]) for p in history_trail]
                pygame.draw.lines(screen, C_BLUE, False, pts_screen, 1)

            # 3. Draw Planned Path (Yellow)
            if len(path_points) > 1:
                pts_screen = [world_to_screen(p[0], p[1]) for p in path_points]
                pygame.draw.lines(screen, C_YELLOW, False, pts_screen, 3)
                # Draw small dots for waypoints
                for pt in pts_screen:
                    pygame.draw.circle(screen, C_YELLOW, pt, 2)

            # 4. Draw Goal (Green)
            gx_s, gy_s = world_to_screen(goal_pose['x'], goal_pose['y'])
            # Target Circle
            pygame.draw.circle(screen, C_GREEN, (gx_s, gy_s), 8, 2) 
            pygame.draw.circle(screen, C_GREEN, (gx_s, gy_s), 2) 
            # Orientation Line (User Def: 0=North, 90=West)
            # Math: X = -sin(th), Y = cos(th). Screen Y inverted -> -cos(th)
            ge_x = gx_s - 20 * math.sin(goal_pose['theta']) 
            ge_y = gy_s - 20 * math.cos(goal_pose['theta'])
            pygame.draw.line(screen, C_GREEN, (gx_s, gy_s), (ge_x, ge_y), 2)

            # 5. Draw Car (Red)
            cx_s, cy_s = world_to_screen(car_pose['x'], car_pose['y'])
            pygame.draw.circle(screen, C_RED, (cx_s, cy_s), 7)
            # Orientation
            ce_x = cx_s - 20 * math.sin(car_pose['theta'])
            ce_y = cy_s - 20 * math.cos(car_pose['theta'])
            pygame.draw.line(screen, C_RED, (cx_s, cy_s), (ce_x, ce_y), 3)
            
            # 6. HUD Text Overlay
            status_lines = [
                f"CAR POS : X{car_pose['x']:5.2f} Y{car_pose['y']:5.2f} T{math.degrees(car_pose['theta']):5.1f}°",
                f"GOAL    : X{goal_pose['x']:5.2f} Y{goal_pose['y']:5.2f}",
                f"STATUS  : {'NAVIGATING' if len(path_points) > 0 else 'IDLE'}",
                f"PATH LEN: {len(path_points)}"
            ]
            
            for i, line in enumerate(status_lines):
                # Draw text with black shadow for readability
                txt_shad = font.render(line, True, C_BLACK)
                txt_main = font.render(line, True, C_YELLOW)
                screen.blit(txt_shad, (12, 12 + i * 20))
                screen.blit(txt_main, (10, 10 + i * 20))

            pygame.display.flip()

        except socket.timeout:
            pass # No data received, keep rendering last frame
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
            break

    client.close()
    pygame.quit()

if __name__ == "__main__":
    main()

