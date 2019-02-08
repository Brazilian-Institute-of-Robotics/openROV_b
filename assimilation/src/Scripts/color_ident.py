#!/usr/bin/env python

import roslib
import cv2
import numpy as np 
import rospy as rp
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import camera


class color_ident():
    def __init__(self):
        self.run = camera.camera()

    def convert_HSV(self):
        img = self.run.get()
        cv2.imshow("Rexrov camera", img)
        cv2.waitKey(3)
        
def main():
    
    rp.init_node("camera_node",anonymous=True)
    run = color_ident()
    while(not rp.is_shutdown()):
        if(run.run.is_ready()):
            run.convert_HSV()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
