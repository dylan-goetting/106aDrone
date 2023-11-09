import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from djitellopy import Tello
import time
from cv_bridge import CvBridge

def talker():

    pub = rospy.Publisher('feed_pub', Image, queue_size=10)

    r = rospy.Rate(10) # 10hz
    tello = Tello()
    tello.connect()
    tello.streamon()

    frame_read = tello.get_frame_read()
    br = CvBridge()
    
    while not rospy.is_shutdown():

        frame = frame_read.frame
        if frame is not None:
            imgmsg = br.cv2_to_imgmsg(frame)    
            # image = Image()
            # image.header = ...
            # image.height = 300
            # image.width = 400
            # image.encoding = ""
            # image.step = 400
            # image.data = frame
            
            pub.publish(imgmsg)
            print(rospy.get_name() + ": I sent \"%s\"" % imgmsg)
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()

    frame_read.stop()
    print('test')
    tello.streamoff()
            
if __name__ == '__main__':

    rospy.init_node('drone_image', anonymous=True)
    try:
        talker()
    except rospy.ROSInterruptException: pass