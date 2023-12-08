from multiprocessing import Process, Value
from dataclasses import dataclass
import cv2
import pickle
import numpy as np
import time
import ctypes
import sys
import os

class AsyncPositionStream:
    """
    Runs in a different process, publishing position to a shared list
    """

    HOST = "0.0.0.0"
    PORT = 11111
    ADDRESS = f"udp://{HOST}:{PORT}"

    def __init__(self, render=False):

        with open("calibration.pkl", "rb") as f:
            self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = pickle.load(
                f)
        self.render = render
        self.rvec = [0] * 3
        self.tvec = [0] * 3
        self.s_rvec = [
            Value(ctypes.c_double, 0),
            Value(ctypes.c_double, 0),
            Value(ctypes.c_double, 0),
        ]
        self.s_tvec = [
            Value(ctypes.c_double, 0),
            Value(ctypes.c_double, 0),
            Value(ctypes.c_double, 0),
        ]
        self.s_fps = Value(ctypes.c_double, 0)
        self.fps = -1
        self.is_child = False
        self._process = Process(target=self._run)
        self._process.start()

    def _run(self):

        self.is_child = True
        capture = cv2.VideoCapture(self.ADDRESS)
        # get first valid frame - takes a while
        ok, _ = capture.read()
        perf_start = time.perf_counter()
        iter = 0
        print("purging")
        for _ in range(100):
            _, _ = capture.read()
        print("starting position stream")
        while ok:
            iter += 1
            ok, frame = capture.read()

            _time = capture.get(cv2.CAP_PROP_POS_MSEC)
            self.rvec, self.tvec = self._parse_frame(frame)
            self.fps = (time.perf_counter() - perf_start) / iter
            self.sync()
            if self.render:
                cv2.imshow("Video Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    def _detect_markers(self, frame):
        return cv2.aruco.detectMarkers(
            frame,
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100),
            parameters=cv2.aruco.DetectorParameters(),
        )

    def _parse_frame(self, frame):
        (all_corners, _, _) = self._detect_markers(frame)

        # no op if none are detected
        if len(all_corners) == 0:
            return self.rvec, self.tvec

        corners = all_corners[0].reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            all_corners[0], 15, self.mtx, self.dist
        )

        if self.render:
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            cv2.drawFrameAxes(frame, self.mtx, self.dist, rvec, tvec, 15)

        tvec = tvec.squeeze()
        rvec = rvec.squeeze()
        return tvec, rvec

    def sync(self):
        if self.is_child:
            for i in range(3):
                self.s_rvec[i].value = self.rvec[i]
                self.s_tvec[i].value = self.tvec[i]
            self.s_fps.value = self.fps
        else:
            for i in range(3):
                self.rvec[i] = self.s_rvec[i].value
                self.tvec[i] = self.s_tvec[i].value
            self.fps = self.s_fps.value

    def pos(self):
        self.sync()
        return self.rvec, self.tvec

    def get_fps(self):
        self.sync()
        return self.fps

    def close(self):
        self._process.terminate()
        self._process.join()
