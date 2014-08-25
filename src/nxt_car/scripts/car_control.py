#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from nxt_msgs.msg import JointCommand
from std_msgs.msg import String


class CarControl(object):
    def __init__(self):
        #self.sub = rospy.Subscriber('joint_state', JointState, self.callback)
        self.pub = rospy.Publisher('car_control', String)
        #self.move_start = None

    def callback(self, data):
        #rospy.loginfo("data len is %d" % len(data.name))

        for i in xrange(0, len(data.name)):
            #rospy.loginfo("data: %f %f" % (data.position[i], data.velocity[i]))

            self.joint_state = data.position[i]

        if self.move_start is not None:
            rospy.loginfo("data: %f %f" % (self.joint_state, self.move_start))
            if self.joint_state - self.move_start > 2:
                self.brake()

    def status(self):
        rospy.loginfo("data: %f" % (self.joint_state))

    def move_fwd(self):
        self.pub.publish('move_fwd')

def main():
    rospy.init_node("remote_control", anonymous=True, log_level=rospy.INFO)

    car_control = CarControl()

    while not rospy.is_shutdown():
        input = raw_input()
        if input == '\x1b[A':
            print "car.move_fwd()"
            car_control.move_fwd()
        if input == '\x1b[C':
            print "car.move_right()"
        if input == '\x1b[D':
            print "car.move_left()"
        if input == '\x1b[B':
            print "car.move_bwd()"

        if input == 's':
            car_control.status()

if __name__ == '__main__':
    main()
