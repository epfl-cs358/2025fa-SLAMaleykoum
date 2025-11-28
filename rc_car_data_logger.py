import socket
import csv
import time
import re
from datetime import datetime

# --- Configuration ---
# You might need to change the HOST IP. If the ESP32 is running in Access Point 
# mode (as suggested by "LIDAR_AP" in your C++ code), the host IP is typically 
# the ESP32's gateway, often 192.168.4.1.
HOST = '192.168.4.1'  # Change this to your RC car's ESP32 IP address
PORT = 9000           # Matches the TCP_PORT in your C++ code
OUTPUT_FILE = 'rc_car_telemetry.csv'
MAX_RETRIES = 5
RECONNECT_DELAY = 5 # seconds

def parse_telemetry_line(line):
    # Format: 
    # CMD: v=0.180 d=90.000 | P: Kp=0.90 Ld=0.25 | T: LX=0.500 LY=1.000 | POSE: X=0.00 Y=0.00 Yaw=0.00 Vel=0.000
    
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
        except ValueError as e:
            print(f"Error converting data: {e} in line: {line.strip()}")
            return None
    return None

def connect_and_log(host, port, filename):
    try:
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = [
                'timestamp_utc', 'time_s', 
                'X_Current', 'Y_Current', 'Yaw_Current', 'Velocity_Current',
                'v_target_CMD', 'delta_target_CMD',
                'Lookahead_X', 'Lookahead_Y', 'Kp', 'Ld'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            print(f"✅ CSV file created: {filename}")

    except IOError as e:
        print(f"FATAL: {e}")
        return

    retries = 0
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
                data = sock.recv(1024).decode('utf-8')
                if not data: break
                buffer += data
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    parsed_data = parse_telemetry_line(line)
                    
                    if parsed_data:
                        current_time = time.time()
                        row = {
                            'timestamp_utc': datetime.utcfromtimestamp(current_time).isoformat(),
                            'time_s': round(current_time - start_time, 3),
                            'X_Current': parsed_data['X'],
                            'Y_Current': parsed_data['Y'],
                            'Yaw_Current': parsed_data['Yaw'],
                            'Velocity_Current': parsed_data['Vel'],
                            'v_target_CMD': parsed_data['v_target'],
                            'delta_target_CMD': parsed_data['delta_target'],
                            'Lookahead_X': parsed_data['Lookahead_X'],
                            'Lookahead_Y': parsed_data['Lookahead_Y'],
                            'Kp': parsed_data['Kp'],
                            'Ld': parsed_data['Ld'],
                        }
                        
                        with open(filename, 'a', newline='') as csvfile_append:
                            writer_append = csv.DictWriter(csvfile_append, fieldnames=fieldnames)
                            writer_append.writerow(row)
                        
                        print(f"[{row['time_s']}s] Pose({row['X_Current']:.2f}, {row['Y_Current']:.2f}) -> LX: {row['Lookahead_X']:.2f}, LY: {row['Lookahead_Y']:.2f}")
        
        except Exception as e:
            print(f"Error: {e}")
        finally:
            sock.close()
            retries += 1
            time.sleep(RECONNECT_DELAY)

if __name__ == "__main__":
    connect_and_log(HOST, PORT, OUTPUT_FILE)