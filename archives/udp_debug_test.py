#!/usr/bin/env python3
# Simple UDP debug receiver - just prints raw data to see what's coming through

import socket

UDP_PORT = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', UDP_PORT))

print(f"Listening on UDP port {UDP_PORT}...")
print("Press Ctrl+C to stop\n")

packet_count = 0
line_count = 0

try:
    while True:
        data, addr = sock.recvfrom(65535)
        packet_count += 1
        
        decoded = data.decode('utf-8', errors='ignore')
        lines = decoded.strip().split('\n')
        line_count += len(lines)
        
        print(f"\n=== Packet {packet_count} from {addr} ({len(data)} bytes, {len(lines)} lines) ===")
        
        # Print first 5 and last 5 lines of each packet
        if len(lines) <= 10:
            for line in lines:
                print(f"  {line}")
        else:
            for line in lines[:5]:
                print(f"  {line}")
            print(f"  ... ({len(lines)-10} more lines) ...")
            for line in lines[-5:]:
                print(f"  {line}")
        
        # Parse a sample line to check format
        if lines:
            try:
                angle_str, dist_str = lines[0].split(",")
                angle = float(angle_str)
                dist = float(dist_str)
                print(f"\n  ✓ Sample parsed: angle={angle:.2f}°, distance={dist:.0f}mm")
            except:
                print(f"\n  ✗ Failed to parse: '{lines[0]}'")
        
        print(f"\nTotal: {packet_count} packets, {line_count} lines received")

except KeyboardInterrupt:
    print(f"\n\nStopped. Received {packet_count} packets, {line_count} total lines")
    sock.close()
