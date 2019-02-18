#!/usr/bin/env python

import roslib
import cv2
import numpy 
import rospy 
import camera
from cv2 import aruco


class color_ident():

    def __init__(self):
        self.start = camera.Launch()
    
    def convert_to_gray(self):
        
        self.gray = cv2.cvtColor(self.start.cv_image, cv2.COLOR_BGR2GRAY)
    
    def adds(self):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(self.gray, aruco_dict, parameters=parameters)
        self.frame_markers = aruco.drawDetectedMarkers(self.start.cv_image, corners, ids)

    def show_img(self):
        img = self.start.get()
        rospy.loginfo_throttle(60,"Camera started")
        cv2.imshow("Tag ident", self.frame_markers)
        cv2.imshow("Rexrov Camera", img)
        cv2.waitKey(3)
        
def main():
    
    rospy.init_node("camera_node",anonymous=True)
    run = color_ident()

    while(not rospy.is_shutdown()):

        if(run.start.is_ready()):

            run.convert_to_gray()
            run.adds()
            run.show_img()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
