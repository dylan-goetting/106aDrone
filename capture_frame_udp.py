import cv2
import time
import socket

local_addr = ('', 11111)
# tello_addr = ('192.168.10.1', 11111)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(local_addr)

iter = 30 * 5

print("starting capture frames")
perf_start = time.perf_counter()
for _ in range(iter):
    while True:
        buf, _ = sock.recvfrom(2048)
        if len(buf) != 1460:
            break
    print("frame recieved")
perf_end = time.perf_counter()

print(f"FPS: {iter / (perf_end - perf_start)}")
