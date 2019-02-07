#!/usr/bin/env python

import roslib
import cv2
import numpy as np 
import rospy as rp
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class camera(object):

    def __init__(self):
        
        
        self.bridge_object = CvBridge()
        self.image_sub = rp.Subscriber("/rexrov/rexrov/camera/camera_image", Image, self.camera_callback)
        self.close = False


    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.close = True
            rp.logfatal(e)
    

def main():

    rexrov_camera = camera()
    rp.init_node("camera_node",anonymous=True)
    while(not rp.is_shutdown()):
        rp.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

        





        
        
