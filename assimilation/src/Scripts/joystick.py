#! usr/bin/dev python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Joystick():

    def __init__(self):

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self._axes = dict(x=4, y=3, z=1,
                          roll=2, pitch=5, yaw=0,
                          xfast=-1, yfast=-1, zfast=-1,
                          rollfast=-1, pitchfast=-1, yawfast=-1)
        
        self._axes_gain = dict(x=3, y=3, z=0.5,
                               roll=0.5, pitch=0.5, yaw=0.5,
                               xfast=6, yfast=6, zfast=1,
                               rollfast=2, pitchfast=2, yawfast=2)
        
    def joy_callback(self, msg):

        self._axes["x"]



        
        

