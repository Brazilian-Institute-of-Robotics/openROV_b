#!/usr/bin/env python

import cv2
import rospy
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from math import pi


class Pan_tilt():
    def __init__(self):

        self.pan_sub = rospy.Subscriber("/joint1_controller/state", JointState, self.callback_pan)
        self.tilt_sub = rospy.Subscriber("/joint2_controller/state", JointState, self.callback_tilt)

        self.pan_pub = rospy.Publisher("/joint1_controller/command", Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher("/joint2_controller/command", Float64, queue_size=10)     

        self.angle = [0, 0]
        self.motor_on = [False, False]
        self.velocity = [0, 0]
        self.home_done = False
        self.goal = [0, 0]

    def callback_pan(self, data):

        self.goal[0] = data.goal_pos
        self.angle[0] = data.current_pos
        self.velocity[0] = data.velocity
        self.motor_on[0] = data.is_moving

    def callback_tilt(self, data):

        self.angle[1] = data.current_pos
        self.velocity[1] = data.velocity
        self.motor_on[1] = data.is_moving

    def go_home(self):

        pan = Float64()
        tilt = Float64()

        pan.data = 0
        tilt.data = 0

        while self.home_done is False:

            self.pan_pub.publish(pan)
            self.tilt_pub.publish(tilt)

            if (self.angle[0] < 0.0016 and self.angle[1] > -0.0031 and self.motor_on[1] is True):
                self.home_done = True

    def scan(self):

        pan = Float64()
        tilt = Float64()
        
        for x in range(1, 9):
            rospy.sleep(1)
            pan.data = x*(pi/4)
            print (pan.data)
            self.pan_pub.publish(pan)
            rospy.sleep(2)

                
        # Tira foto

def main():
    rospy.init_node("Pan_tilt_scan", anonymous=True)
    run = Pan_tilt()
    run.go_home()
    run.scan()
    rospy.spin()


if __name__ == "__main__":
    main()

    