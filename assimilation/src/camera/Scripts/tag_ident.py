#!/usr/bin/env python

import cv2
import rospy
import camera
import numpy
from cv2 import aruco
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pow,sqrt,pi

class Tag_ident():
    def __init__(self):

        self.start = camera.Launch()
        #Subscribe to the pose_gt node to receive the robot's base positon
        self.real_pose = rospy.Subscriber('pose_gt', Odometry, self.update_position)
        #Publish to the cmd_vel to send the linear and angular robot's speed
        self.speed_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.goal = [0,0,-.5]
        self.pose = [0,0,0]
        (self.roll, self.pitch, self.theta) = (0,0,0)
        self.rotation = 0
        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.005
        self.count = 0

    def convert_to_gray(self):

        self.gray = cv2.cvtColor(self.start.cv_image, cv2.COLOR_BGR2GRAY)

    def ident(self):

        
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(self.gray, aruco_dict, parameters=parameters)
        self.frame_markers = aruco.drawDetectedMarkers(self.start.cv_image, corners, ids)

        if ids is not None:
            if ids[0] == 1:
                self.count +=1
        return self.count


    def show_img(self):
        img = self.start.get()
        rospy.loginfo_throttle(60, "Camera started")
        cv2.imshow("Tag ident", self.frame_markers)
        cv2.waitKey(3)
    
    def update_position(self,msg):
        #Receives the data from the pose_gt topic
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        self.pose[2] = msg.pose.pose.position.z
        self.rotation = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([self.rotation.x,
        self.rotation.y, self.rotation.z, self.rotation.w])

    def run(self):
        if self.count >= 30:
            self.goal = [-10, -10, -10]
            
        print(self.goal)
        #Creates the speed variable with the Twist type to publish on cmd_vel topic
        speed = Twist()
        #Calculates the angle with the arc tangent of X-Y considering the signals
        angle =  atan2(self.goal[1] - self.pose[1], self.goal[0] - self.pose[0])
        #Calculates the difference between the robot's angle and the goal angle
        error_angle = angle - self.theta
        #Calculates the difference between the robot's position and the goal
        error_x = self.goal[0] - self.pose[0]
        error_y = self.goal[1] - self.pose[1]
        error_z = self.goal[2] - self.pose[2] 

        if(error_angle > pi):
            error_angle -= 2*(pi)
        elif error_angle < -pi:
            error_angle += 2*(pi)
        speed.angular.z = error_angle*.5
        if speed.angular.z > 1:
            speed.angular.z = 1
        elif speed.angular.z < -.1:
            speed.angular.z = -1
        if abs(error_x) > self.distance_tolerance:
            speed.linear.x = abs(error_x)*.5
            if speed.linear.x > 1:
                speed.linear.x = 1
        else:
            speed.linear.x = 0
            speed.angular.z = 0
        if error_z > self.distance_tolerance or error_z < -self.distance_tolerance:
            speed.linear.z = error_z
            if error_z > 1:
                speed.linear.z = 1
            elif error_z < -1:
                speed.linear.z = -1
        else:
            speed.linear.z = 0
        self.speed_pub.publish(speed)


def main():

    rospy.init_node("camera_node", anonymous=True)
    run = Tag_ident()
    rate = rospy.Rate(10)

    while(not rospy.is_shutdown()):

        if(run.start.is_ready()):

            run.convert_to_gray()
            run.ident()
            run.show_img()
            run.run()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
