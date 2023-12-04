import cv2
import numpy as np

#rvec = np.array([0.0, np.pi, 0])

def get_yaw_command(rvec):
    rmatrix, _ = cv2.Rodrigues(rvec)
    z_axis = np.array(rmatrix[:, 2]).flatten()
    theta = np.arctan2(z_axis[2], z_axis[0])
    theta = theta + np.pi/2
    if theta >= 2*np.pi:
        theta -= 2*np.pi
    if theta < 0:
        theta += 2*np.pi
    
    theta = np.rad2deg(theta)
    return theta


#print(get_yaw_command(rvec))