from async_video_stream import AsyncPositionStream
from djitellopy import Tello
import multiprocessing
import time
from  calibrate_camera import get_calibration
import cv2
import time
import os
import pickle
import pdb
#from utils import get_yaw_command
import numpy as np

def main():

    def move_y(dist):
        dist = int(dist)
        if dist > 0:
            dist = max(dist, 20)
            dist = min(dist, 500)
            tello.move_down(dist)
        else:
            dist = -dist
            dist = max(dist, 20)
            dist = min(dist, 500)
            tello.move_up(dist)

    def move_x(dist):
        dist = int(dist)
        if dist > 0:
            dist = max(dist, 20)
            dist = min(dist, 500)
            tello.move_right(dist)
        else:
            dist = -dist
            dist = max(dist, 20)
            dist = min(dist, 500)
            tello.move_left(dist)

    def rotate_yaw(theta):
        theta = np.rad2deg(theta)

        if theta > 180:
            if theta < 360:
                tello.rotate_clockwise(360 - int(theta))
        else:
            if theta > 1:
                tello.rotate_counter_clockwise(int(theta))
            

        return 360 - theta

    def phase_1_control(tvec, rvec, static=False):
        print("[Phase1] SENDING COMMAND \n")

        rmatrix, _ = cv2.Rodrigues(rvec)
        z_axis = np.array(rmatrix[:, 2]).flatten()
        t1 = np.arctan(z_axis[0]/z_axis[2]) # ANGLE TO ROTATE CCW TO BE ALIGNED WITH TAG

        t2 = np.arctan(tvec[0]/tvec[2])
        t3 = np.pi/2 - t1 + t2
        delta_x = -np.sqrt(tvec[0]**2 + tvec[2]**2) * np.cos(t3)
        delta_z = np.sqrt(tvec[0]**2 + tvec[2]**2) * np.sin(t3)
        print('TVEC:', tvec)
        print("Z AXIS:", z_axis)
        print("T1, t2, t3 ", t1, t2, t3)
        print('[Phase1] Delta_y:', tvec[1]+20)
        print('[Phase1] Delta_yaw degrees:', np.rad2deg(t1))
        print("[Phase1] Delta_x:", delta_x)
        print("[Phase1] Delta_z:", delta_z)

        time.sleep(2)

        if not static:
            rotate_yaw(t1)
            time.sleep(1.5)
            if abs(delta_x) > 20:
                move_x(int(delta_x))
            time.sleep(1.5)
            if abs(tvec[1]+20) > 20:
                move_y(tvec[1]+20)
            time.sleep(1.5)


    def phase_2_control(tvec, rvec):
        print("[Phase 2] SENDING COMMAND\n")
        rmatrix, _ = cv2.Rodrigues(rvec)
        z_axis = np.array(rmatrix[:, 2]).flatten()
        t2 = np.arctan(tvec[0]/tvec[2])
        if abs(tvec[1]+20) > 20:
            move_y(tvec[1]+20)
        time.sleep(1)
        rotate_yaw(t2)
        time.sleep(1)
        tello.move_forward(20)

    vs = AsyncPositionStream(True)

    tello = Tello()
    tello.connect()
    tello.streamon()

    static = True
    if not static:
        tello.takeoff()
        tello.move_up(90)

    while True:
        f = cv2.imread("./calibration/chess20.png")
        cv2.imshow('Video Feed', f)
        cmd = input("Enter a command")
        rvec, tvec = vs.pos()
        print(rvec, tvec)
        if cmd == "1":
            phase_1_control(rvec, tvec, static)
        
        time.sleep(1)


if __name__ == '__main__':
    # multiprocessing.set_start_method('spawn')
    # f = cv2.imread("./calibration/chess20.png")
    # cv2.imshow('Video Feed', f)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    main()