#! usr/bin/dev python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

        
def joy_callback(data):

    twist = Twist()
    twist.linear.x = 4*data.axes[7]
    twist.angular.z = 4*data.axes[6]
    cmd_pub.publish(twist)

def start():

    global cmd_pub

    joy_sub = rospy.Subscriber("joy", Joy, joy_callback)
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.init_node('Joy_controller')
    rospy.spin()
        



        



        
        

