import cv2
from djitellopy import Tello
import time
tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

import cv2
import time

# Initialize the BackgroundFrameRead object
frame_reader = frame_read

try:
    while True:
        # Get the current frame
        frame = frame_reader.frame

        # If a frame was received
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
