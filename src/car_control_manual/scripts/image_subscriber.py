#!/usr/bin/python


import roslib, rospy
import sys
import cv2


from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:
    def __init__(self):
        cv2.namedWindow('Image window', 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/decomp_image', numpy_msg, self.ros_to_cv2)

    def ros_to_cv2(self, img):
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            cv2.imshow('Image window', img)
            cv2.waitKey(3)
        except CvBridgeError, e:
            print e


class image_receiver(object):
    def __init__(self):
        cv2.namedWindow('Image window', 1)
        # self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_publisher', String, self.callback)

    def callback(self, data):

        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        try:
            # cv_image = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
            cv2.imshow('Image window', image_np)
            cv2.waitKey(3)
        except CvBridgeError, e:
            print e


def main():
    ir = image_receiver()
    rospy.init_node('image_convert', anonymous = True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting downs"
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
