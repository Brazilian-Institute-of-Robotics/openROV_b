#! usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy


class Camera():
    def __init__(self):

        self.bridge_object = CvBridge()
        self.video = cv2.VideoCapture()
        self.pub_video = rospy.Publisher("/image_raw", Image, queue_size=10)
        self.frame = None

    def video_cap(self):

        __, self.frame = self.video.read()

    def video(self):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(self.frame, desired_encoding="bgr8")
            self.pub_video.publish(cv_image)

        except CvBridgeError as e:

            self.close = True
            rospy.logfatal(e)


if __name__ == "__main__":

    rospy.init_node("Camera", anonymous=True)
    cam = Camera()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        cam.video_cap()
        cam.video()
        rate.sleep()
