from djitellopy import Tello
import time
from  calibrate_camera import get_calibration
import cv2
import time
import os
import pickle
import pdb
#from utils import get_yaw_command
import numpy as np

with open('calibration.pkl', 'rb') as f:
    ret, mtx, dist, rvecs, tvecs = pickle.load(f)

static = True
tello = Tello()
tello.connect()
tello.streamon()
if not static:
    tello.takeoff()
    tello.move_up(90)
frame_read = tello.get_frame_read()
# Initialize the BackgroundFrameRead object
frame_reader = frame_read

i = 0
j = 0

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


print("--------------------------START----------------------")
buf = []
try:
    t = time.time()
    while True:
        i += 1
        # Get the current frame
        frame = frame_reader.frame
        #if i % 20 == 0:
        #    cv2.imwrite(f"calibration/chess{i}.png", frame)
        # If a frame was received
        last_y_err = 0
        (all_corners, ids, rejected) = cv2.aruco.detectMarkers(frame, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100), parameters=cv2.aruco.DetectorParameters())
        if len(all_corners) > 0:
            j += 1
            corners = all_corners[0].reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            #cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(all_corners[0], 15, mtx, dist)
            cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 15)
            tvec = tvec.squeeze()
            rvec = rvec.squeeze()
            rmatrix, _ = cv2.Rodrigues(rvec)
            z_axis = np.array(rmatrix[:, 2]).flatten()
            buf.append([tvec, z_axis])
            print(f'Found AR tag\n')
            #print("TVEC:", tvec)
            phase_1_control(tvec, rvec, static)

            t = time.time()

        if frame is not None and i % 3 == 0:
            # Display the frame
            cv2.imshow('Video Feed', frame)
            #tello.send_control_command('command')
        # If 'q' is pressed on the keyboard, break the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Optional: sleep for a short period to control the rate of the video display
        time.sleep(0.1)

finally:
    print('LATENCY:' , i/(time.time() - t))
    # Stop the frame reader
    frame_reader.stop()
    # Close the OpenCV window
    cv2.destroyAllWindows()

print('test')
pdb.set_trace()
tello.streamoff()
