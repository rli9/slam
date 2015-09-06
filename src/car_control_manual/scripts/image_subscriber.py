#!/usr/bin/python


import rospy
import cv2

from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import Queue
from sensor_msgs.msg import Image


# for mobile example convert
class ImageConverter:
    def __init__(self):
        cv2.namedWindow('Image window', 1)
        self.bridge = CvBridge()
        self.image_que = None
        self.image_sub = rospy.Subscriber('/camera/decomp_image', Image, self.ros_to_cv2_show)
        self.take_photo_sub = rospy.Subscriber('car_control_manual/take_photo', String, self.take_photo)

    def ros_to_cv2_show(self, img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            try:
                self.image_que = cv_image
            except Queue.Full, e:
                # self.image_que.get()
                print("Image Queue Full %s" % e)
            cv2.imshow('Image window', cv_image)
            cv2.waitKey(3)
        except CvBridgeError, e:
            print e

    def take_photo(self, data):
        print("Recevie Take photo Cmd : %s " % data.data)
        if data.data == u'on':
            print("Take Photo Start.")

            try:
                cv_image = self.image_que
                cv2.imwrite("save.png", cv_image)
            except Queue.Empty, e:
                print("Image Queue Empty")


            print("Take Photo End.")
        else:
            print("Turn off photo.")



# for self defined String format Image stream
class ImageReceiver(object):
    def __init__(self):
        cv2.namedWindow('Image window', 1)
        # self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/decomp_image', String, self.callback)

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
    ir = ImageConverter()
    rospy.init_node('image_convert', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting downs"
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
