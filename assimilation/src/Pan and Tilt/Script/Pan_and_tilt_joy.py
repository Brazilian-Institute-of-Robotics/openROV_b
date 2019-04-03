#!/usr/bin/env python

import rospy
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from math import pi


class Pan_tilt_joy():
    def __init__(self):

        self.pan_sub = rospy.Subscriber("/joint1_controller/state", JointState, self.callback_pan)
        self.tilt_sub = rospy.Subscriber("/joint2_controller/state", JointState, self.callback_tilt)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

        self.pan_pub = rospy.Publisher("/joint1_controller/command", Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher("/joint2_controller/command", Float64, queue_size=10)

        self.pan = 0.0
        self.tilt = 0.0
        self.angle = [0, 0]

    def callback_pan(self, data):

        self.angle[0] = data.current_pos

    def callback_tilt(self, data):

        self.angle[1] = data.current_pos

    def joy_callback(self, data):

        self.pan = data.axes[0]
        self.tilt = data.axes[1]

    def joystick(self):

        pan = Float64()
        tilt = Float64()

        if (self.angle[1] < -pi/4 and self.tilt < 0):
            self.tilt = 0
        elif(self.angle[1] > pi/4 and self.tilt > 0):
            self.tilt = 0

        pan.data = self.pan
        tilt.data = self.tilt

        self.pan_pub.publish(pan)
        self.tilt_pub.publish(tilt)


if __name__ == "__main__":

    rospy.init_node("Pan_tilt_joy", anonymous=True)
    run = Pan_tilt_joy()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        run.joystick()
        rate.sleep()
