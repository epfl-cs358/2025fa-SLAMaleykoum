import socket
import csv
import time
import re
from datetime import datetime

HOST = '192.168.4.1'
PORT = 9000
TELEMETRY_FILE = 'rc_car_telemetry.csv'
PATHS_FILE = 'rc_car_paths.csv'
MAX_RETRIES = 5
RECONNECT_DELAY = 5 

def parse_telemetry_line(line):
    # Regex for standard telemetry
    pattern = re.compile(
        r"CMD: v=([-\d.]+)\s+d=([-\d.]+)\s+\|\s+"
        r"P: Kp=([-\d.]+)\s+Ld=([-\d.]+)\s+\|\s+"
        r"T: LX=([-\d.]+)\s+LY=([-\d.]+)\s+\|\s+"
        r"POSE: X=([-\d.]+)\s+Y=([-\d.]+)\s+Yaw=([-\d.]+)\s+Vel=([-\d.]+)"
    )
    match = pattern.search(line)
    if match:
        try:
            return {
                'v_target': float(match.group(1)),
                'delta_target': float(match.group(2)),
                'Kp': float(match.group(3)),
                'Ld': float(match.group(4)),
                'Lookahead_X': float(match.group(5)),
                'Lookahead_Y': float(match.group(6)),
                'X': float(match.group(7)),
                'Y': float(match.group(8)),
                'Yaw': float(match.group(9)),
                'Vel': float(match.group(10)),
            }
        except ValueError:
            return None
    return None

def connect_and_log(host, port):
    # Initialize files
    with open(TELEMETRY_FILE, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'timestamp_utc', 'time_s', 'X_Current', 'Y_Current', 'Yaw_Current', 
            'Velocity_Current', 'v_target_CMD', 'delta_target_CMD', 
            'Lookahead_X', 'Lookahead_Y', 'Kp', 'Ld'
        ])
        writer.writeheader()
    
    with open(PATHS_FILE, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['path_id', 'index', 'x', 'y'])
        writer.writeheader()

    retries = 0
    current_path_id = -1

    while retries < MAX_RETRIES:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        try:
            print(f"Connecting to {host}:{port}...")
            sock.connect((host, port))
            print("🟢 Connected! Sending 'start'...")
            sock.sendall(b'start\n')
            
            buffer = ""
            start_time = time.time()
            
            while True:
                try:
                    data = sock.recv(2048).decode('utf-8')
                    if not data: break
                    buffer += data
                except socket.timeout:
                    continue
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line: continue

                    # --- Check for Path Messages ---
                    if line.startswith("PATH_START"):
                        # Format: PATH_START: ID=1 LEN=20
                        try:
                            parts = line.split()
                            pid_str = parts[1].split('=')[1]
                            current_path_id = int(pid_str)
                            print(f"🔵 Receiving New Path ID: {current_path_id}")
                        except:
                            print("Error parsing PATH_START")
                    
                    elif line.startswith("WP"):
                        # Format: WP: 0 0.000 0.000
                        try:
                            parts = line.split()
                            idx = int(parts[1])
                            px = float(parts[2])
                            py = float(parts[3])
                            
                            # Save Waypoint
                            with open(PATHS_FILE, 'a', newline='') as f:
                                w = csv.DictWriter(f, fieldnames=['path_id', 'index', 'x', 'y'])
                                w.writerow({'path_id': current_path_id, 'index': idx, 'x': px, 'y': py})
                        except:
                            pass
                            
                    elif line.startswith("PATH_END"):
                        print(f"✅ Path {current_path_id} received completely.")

                    # --- Check for Telemetry ---
                    elif line.startswith("CMD"):
                        parsed = parse_telemetry_line(line)
                        if parsed:
                            current_time = time.time()
                            row = {
                                'timestamp_utc': datetime.utcfromtimestamp(current_time).isoformat(),
                                'time_s': round(current_time - start_time, 3),
                                'X_Current': parsed['X'], 'Y_Current': parsed['Y'],
                                'Yaw_Current': parsed['Yaw'], 'Velocity_Current': parsed['Vel'],
                                'v_target_CMD': parsed['v_target'], 'delta_target_CMD': parsed['delta_target'],
                                'Lookahead_X': parsed['Lookahead_X'], 'Lookahead_Y': parsed['Lookahead_Y'],
                                'Kp': parsed['Kp'], 'Ld': parsed['Ld'],
                            }
                            with open(TELEMETRY_FILE, 'a', newline='') as f:
                                w = csv.DictWriter(f, fieldnames=row.keys())
                                w.writerow(row)
                            print(f"[{row['time_s']}s] Pose({row['X_Current']:.2f}, {row['Y_Current']:.2f})")

        except Exception as e:
            print(f"Error: {e}")
            sock.close()
            retries += 1
            time.sleep(RECONNECT_DELAY)

if __name__ == "__main__":
    connect_and_log(HOST, PORT)

    