import cv2
from djitellopy import Tello
import time
tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()
from  calibrate_camera import get_calibration
import cv2
import time
import os

# Initialize the BackgroundFrameRead object
frame_reader = frame_read
ret, mtx, dist, rvecs, tvecs = get_calibration()
i = 0
try:
    while True:
        i += 1
        # Get the current frame
        frame = frame_reader.frame
        #if i % 20 == 0:
        #    cv2.imwrite(f"calibration/chess{i}.png", frame)
        # If a frame was received
        (all_corners, ids, rejected) = cv2.aruco.detectMarkers(frame, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100), parameters=cv2.aruco.DetectorParameters())
        if len(all_corners) > 0:
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
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(all_corners[0], 0.02, mtx, dist)
            cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.02)

        if frame is not None:
            # Display the frame
            cv2.imshow('Video Feed', frame)
        # If 'q' is pressed on the keyboard, break the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Optional: sleep for a short period to control the rate of the video display
        time.sleep(0.1)

finally:
    # Stop the frame reader
    frame_reader.stop()

    # Close the OpenCV window
    cv2.destroyAllWindows()

print('test')
tello.streamoff()
