#!/usr/bin/python
__author__ = 'flex'

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2
import rospy, roslib

class ImagePublisher(object):
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.image_pub = rospy.Publisher('image_publisher', String, queue_size = 1)

        self.cap = cv2.VideoCapture(1)

        try:
            self.cap.read()
        except:
            self.cap = cv2.VideoCapture(0)

    def publish(self,raw_img):
        msg = CompressedImage()
        msg.header.stamp =  rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = np.array(cv2.imencode('.jpg', raw_img)[1]).tostring()
        #TODO compile sensor_msg error , use String instead
        self.image_pub.publish(msg.data)

    def run(self):
        while True:
            ret, frame = self.cap.read()
            self.publish(frame)
            # cv2.imshow(frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cap.release()


if __name__ == "__main__":
    ip = ImagePublisher()
    ip.run()



