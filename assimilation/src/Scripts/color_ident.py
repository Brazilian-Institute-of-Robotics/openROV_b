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
        self.start = camera.launch()
        self.descentre = 160
        self.rows_to_watch = 20
        

    def convert_to_HSV(self):
        
        hsv = cv2.cvtColor(self.start.cv_image, cv2.COLOR_BGR2HSV)
        print(hsv)

    def show_img(self):
        img = self.start.get()
        cv2.imshow("Rexrov camera", img)
        cv2.waitKey(3)
        
def main():
    
    rp.init_node("camera_node",anonymous=True)
    run = color_ident()
    while(not rp.is_shutdown()):
        if(run.start.is_ready()):
            run.show_img()
            run.convert_to_HSV()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
