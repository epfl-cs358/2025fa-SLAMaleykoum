import socket
import struct

HOST = "192.168.4.1"
PORT = 9000

fmt = "<fQfQ"
packet_size = struct.calcsize(fmt)

print("Connecting...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
print("Connected.\n")

s.setblocking(False)

buffer = b""

while True:
    try:
        chunk = s.recv(1024)
    except BlockingIOError:
        chunk = None

    if chunk:
        buffer = chunk

        # 1) TRAITEMENT DES MESSAGES TEXTE
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            try:
                txt = line.decode("utf-8")
                print("[TXT]", txt)
            except:
                pass  # ce n'était pas du texte

    # 2) TRAITEMENT DU binaire
    if len(buffer) >= packet_size:
        bin_packet = buffer[:packet_size]
        buffer = buffer[packet_size:]

        pos1, tsf1, pos2, tsf2 = struct.unpack(fmt, bin_packet)

        tsf1_ms = tsf1 / 1000.0
        tsf2_ms = tsf2 / 1000.0
        delta = abs(tsf1_ms - tsf2_ms)

        print(f"[BIN] ESP1 loop={pos1:6.1f}, tsf={tsf1_ms:12.3f} ms |  "
              f"ESP2 loop={pos2:6.1f}, tsf={tsf2_ms:12.3f} ms |  "
              f"Δclock = {delta:.6f} ms")