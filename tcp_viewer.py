#!/usr/bin/env python3
"""
ESP32 Path Planner TCP Viewer
Connects to ESP32 and displays maps + waypoints in terminal

Usage:
    python tcp_viewer.py                    # Auto-connect to 192.168.4.1:9000
    python tcp_viewer.py 192.168.1.100      # Custom IP
    python tcp_viewer.py 192.168.1.100 8080 # Custom IP and port

Controls:
    Press ENTER to cycle through tests
    Type 0-4 to jump to specific test
    Ctrl+C to exit
"""

import socket
import sys
import select
import os
import time

# ANSI color codes for terminal
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BG_BLACK = '\033[40m'
    BG_GREEN = '\033[42m'
    BG_RED = '\033[41m'

def clear_screen():
    """Clear terminal screen"""
    os.system('cls' if os.name == 'nt' else 'clear')

def colorize_map(line):
    """Add colors to ASCII map"""
    colored = ""
    for char in line:
        if char == 'S':
            colored += f"{Colors.BG_GREEN}{Colors.BOLD}{Colors.WHITE}{char}{Colors.RESET}"
        elif char == 'G':
            colored += f"{Colors.BG_RED}{Colors.BOLD}{Colors.WHITE}{char}{Colors.RESET}"
        elif char == '.':
            colored += f"{Colors.GREEN}{Colors.BOLD}{char}{Colors.RESET}"
        elif char == '#':
            colored += f"{Colors.WHITE}{Colors.BG_BLACK}{char}{Colors.RESET}"
        elif char == '%':
            colored += f"{Colors.YELLOW}{char}{Colors.RESET}"
        else:
            colored += char
    return colored

class ESP32Viewer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False
        
    def connect(self):
        """Connect to ESP32 TCP server"""
        print(f"{Colors.CYAN}[TCP] Connecting to {self.host}:{self.port}...{Colors.RESET}")
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(None)  # Set back to blocking
            
            self.connected = True
            print(f"{Colors.GREEN}[TCP] ✓ Connected!{Colors.RESET}\n")
            return True
            
        except socket.timeout:
            print(f"{Colors.RED}[ERROR] Connection timeout!{Colors.RESET}")
            print(f"{Colors.YELLOW}[HINT] Make sure you're connected to WiFi: LIDAR_AP{Colors.RESET}")
            return False
            
        except ConnectionRefusedError:
            print(f"{Colors.RED}[ERROR] Connection refused!{Colors.RESET}")
            print(f"{Colors.YELLOW}[HINT] Is the ESP32 powered on and WiFi AP running?{Colors.RESET}")
            return False
            
        except Exception as e:
            print(f"{Colors.RED}[ERROR] {e}{Colors.RESET}")
            return False
    
    def receive_data(self):
        """Receive and display data from ESP32"""
        buffer = ""
        in_map = False
        
        try:
            while self.connected:
                # Check if there's data to receive
                ready = select.select([self.sock], [], [], 0.1)
                
                if ready[0]:
                    data = self.sock.recv(4096).decode('utf-8', errors='ignore')
                    
                    if not data:
                        print(f"\n{Colors.RED}[TCP] Connection closed by ESP32{Colors.RESET}")
                        self.connected = False
                        break
                    
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        
                        # Detect map section for coloring
                        if '╔' in line or '║' in line:
                            in_map = True
                        
                        # Colorize map lines
                        if in_map and any(c in line for c in ['S', 'G', '.', '#', '%']):
                            print(colorize_map(line))
                        else:
                            print(line)
                        
                        if '╚' in line:
                            in_map = False
                
                # Check for user input (non-blocking)
                if sys.platform != 'win32':  # Unix-like systems
                    ready_stdin = select.select([sys.stdin], [], [], 0)[0]
                    if ready_stdin:
                        user_input = sys.stdin.readline().strip()
                        self.send_command(user_input)
                else:  # Windows - use simple input
                    # On Windows, we can't do non-blocking stdin easily
                    # So we just keep receiving
                    pass
                    
        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}[USER] Interrupt received{Colors.RESET}")
        except Exception as e:
            print(f"\n{Colors.RED}[ERROR] {e}{Colors.RESET}")
        finally:
            self.disconnect()
    
    def send_command(self, cmd):
        """Send command to ESP32"""
        try:
            self.sock.sendall((cmd + '\n').encode())
        except Exception as e:
            print(f"{Colors.RED}[ERROR] Failed to send: {e}{Colors.RESET}")
    
    def disconnect(self):
        """Close connection"""
        if self.sock:
            self.sock.close()
        self.connected = False
        print(f"\n{Colors.CYAN}[TCP] Disconnected{Colors.RESET}")

    def interactive_mode(self):
        """Interactive mode with user input"""
        print(f"\n{Colors.BOLD}{Colors.CYAN}═══════════════════════════════════════════════════════{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.WHITE}    ESP32 Path Planner - Interactive Mode{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.CYAN}═══════════════════════════════════════════════════════{Colors.RESET}\n")
        print(f"{Colors.GREEN}Commands:{Colors.RESET}")
        print(f"  {Colors.YELLOW}ENTER{Colors.RESET}  - Next test scenario")
        print(f"  {Colors.YELLOW}0-4{Colors.RESET}    - Jump to specific test")
        print(f"  {Colors.YELLOW}q{Colors.RESET}      - Quit")
        print(f"{Colors.CYAN}═══════════════════════════════════════════════════════{Colors.RESET}\n")
        
        try:
            while self.connected:
                # Receive data
                ready = select.select([self.sock], [], [], 0.1)
                if ready[0]:
                    data = self.sock.recv(4096).decode('utf-8', errors='ignore')
                    if not data:
                        print(f"\n{Colors.RED}[TCP] Connection closed{Colors.RESET}")
                        break
                    
                    # Print with colors
                    for line in data.split('\n'):
                        if any(c in line for c in ['S', 'G', '.', '#', '%']):
                            print(colorize_map(line))
                        else:
                            print(line)
                
                # Get user input (blocking on Windows, non-blocking on Unix)
                if sys.platform != 'win32':
                    ready_stdin = select.select([sys.stdin], [], [], 0.1)[0]
                    if ready_stdin:
                        cmd = sys.stdin.readline().strip()
                        if cmd.lower() == 'q':
                            break
                        self.send_command(cmd)
                
        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}[EXIT] User interrupt{Colors.RESET}")
        finally:
            self.disconnect()

def main():
    # Parse arguments
    host = '192.168.4.1'  # Default ESP32 AP IP
    port = 9000
    
    if len(sys.argv) > 1:
        host = sys.argv[1]
    if len(sys.argv) > 2:
        port = int(sys.argv[2])
    
    # Print banner
    clear_screen()
    print(f"\n{Colors.BOLD}{Colors.CYAN}╔══════════════════════════════════════════════════════════╗{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}║{Colors.RESET}  {Colors.BOLD}{Colors.WHITE}ESP32-S3 Path Planner TCP Viewer{Colors.RESET}                  {Colors.BOLD}{Colors.CYAN}║{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}╚══════════════════════════════════════════════════════════╝{Colors.RESET}\n")
    
    # Connection checklist
    print(f"{Colors.YELLOW}[CHECKLIST]{Colors.RESET}")
    print(f"  1. ESP32 powered on? {Colors.GREEN}✓{Colors.RESET}")
    print(f"  2. Connected to WiFi SSID: {Colors.CYAN}LIDAR_AP{Colors.RESET}? {Colors.GREEN}✓{Colors.RESET}")
    print(f"  3. Target: {Colors.CYAN}{host}:{port}{Colors.RESET}\n")
    
    # Create viewer and connect
    viewer = ESP32Viewer(host, port)
    
    if viewer.connect():
        # Wait a bit for welcome message
        time.sleep(0.5)
        
        # Start interactive mode
        viewer.interactive_mode()
    else:
        print(f"\n{Colors.RED}[FAILED] Could not connect to ESP32{Colors.RESET}")
        print(f"\n{Colors.YELLOW}Troubleshooting:{Colors.RESET}")
        print(f"  1. Check WiFi connection: Are you connected to 'LIDAR_AP'?")
        print(f"  2. Verify ESP32 IP: Try pinging {host}")
        print(f"  3. Check ESP32 serial output for actual IP address")
        print(f"  4. Firewall: Make sure port {port} is not blocked")
        sys.exit(1)

if __name__ == '__main__':
    main()