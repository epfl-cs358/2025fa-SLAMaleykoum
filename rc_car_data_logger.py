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
    """
    Parses a single line of telemetry data from the car.
    Expected format: "CMD: v_target=0.180 delta_target=90.000 | POSE: X=0.00 Y=0.00 Yaw=0.00 Vel=0.000\n"
    
    Returns a dictionary of parsed values or None if parsing fails.
    """
    
    # Define a robust regular expression to capture all float values
    # Group 1: v_target, Group 2: delta_target
    # Group 3: X, Group 4: Y, Group 5: Yaw, Group 6: Vel
    # Note: We look for a pattern that captures floating point numbers (e.g., -1.23, 4.00, .5)
    pattern = re.compile(
        r"CMD: v_target=([-\d.]+)\s+"
        r"delta_target=([-\d.]+)\s+\|\s+"
        r"POSE: X=([-\d.]+)\s+"
        r"Y=([-\d.]+)\s+"
        r"Yaw=([-\d.]+)\s+"
        r"Vel=([-\d.]+)"
    )
    
    match = pattern.search(line)
    
    if match:
        try:
            return {
                'v_target': float(match.group(1)),
                'delta_target': float(match.group(2)),
                'X': float(match.group(3)),
                'Y': float(match.group(4)),
                'Yaw': float(match.group(5)),
                'Vel': float(match.group(6)),
            }
        except ValueError as e:
            print(f"Error converting data to float: {e} in line: {line.strip()}")
            return None
    else:
        # This often happens with incomplete or corrupt lines, especially at startup
        # print(f"Warning: Line did not match expected pattern: {line.strip()}")
        return None


def connect_and_log(host, port, filename):
    """
    Connects to the car's TCP server and continuously logs data.
    """
    # Open the CSV file and write the header
    try:
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = [
                'timestamp_utc', 
                'time_s', 
                'X_Current', 'Y_Current', 'Yaw_Current', 'Velocity_Current',
                'v_target_CMD', 'delta_target_CMD'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            print(f"✅ CSV file created: {filename}")

    except IOError as e:
        print(f"FATAL: Could not open or write to file {filename}: {e}")
        return

    # Start main logging loop
    retries = 0
    while retries < MAX_RETRIES:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5) # Set a timeout for connection and receiving

        try:
            print(f"Attempting to connect to {host}:{port} (Attempt {retries + 1}/{MAX_RETRIES})...")
            sock.connect((host, port))
            print("🟢 Connection successful!")
            
            # --- NEW: Send Start Command ---
            print("🚀 Sending 'start' command to car...")
            sock.sendall(b'start\n')
            # -------------------------------

            print("📷 Starting data capture...")
            
            # Reset buffer and initial time on successful connection
            buffer = ""
            start_time = time.time()
            
            while True:
                # Receive data in chunks
                data = sock.recv(1024).decode('utf-8')
                
                if not data:
                    print("Connection closed by the remote host.")
                    break
                
                # Append the new data to the buffer
                buffer += data
                
                # Process the buffer line by line
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    
                    # Parse the line
                    parsed_data = parse_telemetry_line(line)
                    
                    if parsed_data:
                        current_time = time.time()
                        
                        # Prepare row for CSV
                        row = {
                            'timestamp_utc': datetime.utcfromtimestamp(current_time).isoformat(),
                            'time_s': round(current_time - start_time, 3),
                            'X_Current': parsed_data['X'],
                            'Y_Current': parsed_data['Y'],
                            'Yaw_Current': parsed_data['Yaw'],
                            'Velocity_Current': parsed_data['Vel'],
                            'v_target_CMD': parsed_data['v_target'],
                            'delta_target_CMD': parsed_data['delta_target'],
                        }
                        
                        # Write to file (we re-open the file in 'a'ppend mode for each write block)
                        with open(filename, 'a', newline='') as csvfile_append:
                            writer_append = csv.DictWriter(csvfile_append, fieldnames=fieldnames)
                            writer_append.writerow(row)
                            
                        # Print status to console
                        print(f"[{row['time_s']}s] X={row['X_Current']:.2f}, Y={row['Y_Current']:.2f}, V_cmd={row['v_target_CMD']:.3f}")
                        
        
        except socket.timeout:
            print("Connection timeout.")
        except socket.error as e:
            print(f"Socket error: {e}")
        except KeyboardInterrupt:
            print("\n👋 Logging stopped by user.")
            sock.close()
            return
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        finally:
            # Clean up the socket connection before retrying or exiting
            sock.close()
            retries += 1
            if retries < MAX_RETRIES:
                print(f"Retrying connection in {RECONNECT_DELAY} seconds...")
                time.sleep(RECONNECT_DELAY)

    print(f"❌ Failed to connect after {MAX_RETRIES} attempts. Exiting.")


if __name__ == "__main__":
    connect_and_log(HOST, PORT, OUTPUT_FILE)
