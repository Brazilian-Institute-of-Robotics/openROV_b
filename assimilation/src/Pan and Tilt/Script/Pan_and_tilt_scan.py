#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from math import pi


#class Camera():
#    def __init__(self):
#        
#        self.bridge_object = CvBridge()
#        self.sub_video = rospy.Subscriber("/image_raw", Image, self.video_callback)
#
#    def video_callback(self, data):
#        
#        try:
#            self.image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
#
#        except CvBridgeError as e:
#            print(e)
#
#    def show_img(self):
#
#        cv2.imshow("Camera", self.image)
#
#    def save_frame(self, pan, tilt):
#
#        cv2.imwrite("/home/teo/Documentos/Workspaces/rexrov/src/openROV_b/assimilation/src/Pan and Tilt/Data/Pan_and_tilt_frame_" + str(pan) + "/" + str(tilt) + ".png", self.image)
#

class Pan_tilt():
    def __init__(self):

        self.pan_sub = rospy.Subscriber("/joint1_controller/state", JointState, self.callback_pan)
        self.tilt_sub = rospy.Subscriber("/joint2_controller/state", JointState, self.callback_tilt)

        self.pan_pub = rospy.Publisher("/joint1_controller/command", Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher("/joint2_controller/command", Float64, queue_size=10)     

#        self.cam = Camera()
        self.angle = [0, 0]
        self.motor_on = [False, False]
        self.home_done = False
        self.goal = [0, 0]

    def callback_pan(self, data):

        self.angle[0] = data.current_pos
        self.motor_on[0] = data.is_moving

    def callback_tilt(self, data):

        self.angle[1] = data.current_pos
        self.motor_on[1] = data.is_moving

    def publish(self, pan, tilt):

        self.pan_pub.publish(pan)
        self.tilt_pub.publish(tilt)

    def scan(self):

        rospy.sleep(3)

        self.publish(0, 0)
        rospy.sleep(6)

        for x in range(0, 8):

          #  self.cam.save_frame(x, 0)
            rospy.sleep(1)
            self.publish(x*(pi/4), 0)
            rospy.sleep(2)
         #   self.cam.save_frame(x, 1)
            rospy.sleep(1)
            self.publish(x*(pi/4), -pi/4)
            rospy.sleep(2)

        # for y in range(8, 0):
        #
        #    rospy.sleep(1)
        #    self.publish(x*(pi/4), pi/4)
        #    rospy.sleep(2)


if __name__ == "__main__":

    rospy.init_node("Pan_tilt_scan", anonymous=True)

    run = Pan_tilt()
    run.scan()
    rospy.spin()

    