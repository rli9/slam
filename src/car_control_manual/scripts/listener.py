#!/usr/bin/python




import rospy
from std_msgs.msg import String
from car_control_manual.msg import CarControlMsg


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I have heard %s %s", data.x, data.y)

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("car_control_manual", CarControlMsg, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
