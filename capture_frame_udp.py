import cv2
import os
import time
import socket
from async_video_stream import AsyncPositionStream

ITER = 30 * 5

HOST = '0.0.0.0'
PORT = 11111

def capture_udp(iter) -> float:
    local_addr = ('', PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(local_addr)

    print("starting capture frames")
    for _ in range(iter):
        h264_frame = None
        frame = b''
        buf = None
        while buf == None or len(buf) == 1460:
            buf, _ = sock.recvfrom(2048)
            frame += buf
        print("frame")

def capture_cv2(iter) -> float:
    cap = cv2.VideoCapture(f"udp://{HOST}:{PORT}")
    print("starting capture frames")
    ret, frame = cap.read()
    for _ in range(iter):
        ret, frame = cap.read()

def capture_async_pos():
    vs = AsyncPositionStream(False)
    print("starting capture frames")
    for i in range(10000):
        os.system("clear")
        print(vs.pos())

    print(vs.get_fps())


# perf_start = time.perf_counter()
# capture_cv2(ITER)
# perf_end = time.perf_counter()
# print(f"FPS: {ITER / (perf_end - perf_start)}")
capture_async_pos()
