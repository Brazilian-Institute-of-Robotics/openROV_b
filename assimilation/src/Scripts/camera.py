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
        self.ready = False


    def camera_callback(self,data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.ready = True

        except CvBridgeError as e:
            self.close = True
            rp.logfatal(e)
    
    def get(self):
        if (self.ready):
            return self.cv_image
        else:
            empty_img = np.zeros((492,768,3),np.uint8)
            return empty_img

    def is_ready(self):
        return self.ready
def main():

    rp.init_node("camera_node",anonymous=True)
    while(not rp.is_shutdown()):
        rp.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

        





        
        
