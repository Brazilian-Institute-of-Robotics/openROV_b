#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pow,sqrt

goal_x = 0
goal_y = 0
goal_z = 0
x = y = z = theta = 0

def move(msg):
    global x, y, z, theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def set_target(msg):
    global goal_x, goal_y, goal_z
    goal_x = msg.x
    goal_y = msg.y
    goal_z = msg.z

rospy.init_node("speed_controller")
target = rospy.Subscriber('Point',Point, set_target)
real_pose = rospy.Subscriber('pose_gt', Odometry, move)
speedpub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

while not rospy.is_shutdown():
    dist_x = goal_x - x        
    dist_y = goal_y - y        
    dist_z = goal_z - z 
    angle = atan2(dist_y, dist_x)
    
    if (angle - theta) > 0.2:
        speed.angular.z = 0.3
    elif (angle - theta) < -0.2:
        speed.angular.z = -0.3
    else:
        speed.angular.z = 0
    
    if dist_x >= .5 or dist_y >= .5:
        speed.linear.x = .5
    else:
        speed.linear.x = 0
    
    if dist_z >= .5:
        speed.linear.z = 0.5
    else:
        speed.linear.z = 0
    speedpub.publish(speed)
    r.sleep()    
    