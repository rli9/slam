#!/usr/bin/env python

import roslib
roslib.load_manifest('nxt_car')

import rospy
import nxt.locator
from nxt.motor import PORT_A, PORT_B
import nxt.motor
from std_msgs.msg import String


class Car(object):
    def __init__(self):
        nxt.motor.Motor.debug = 1

        self.brick = nxt.locator.find_one_brick()

        self.r_motor = nxt.motor.Motor(self.brick, PORT_A)
        self.l_motor = nxt.motor.Motor(self.brick, PORT_B)

        self.sub = rospy.Subscriber('car_control', String, self.control_callback)

    def control_callback(self, data):
        rospy.loginfo('control_callback(%s)' % data.data)
        getattr(self, data.data)()

    def move_forward(self):
        self.l_motor.weak_turn(64, 360)
        self.r_motor.weak_turn(64, 360)

    def move_backward(self):
        self.l_motor.weak_turn(-64, 360)
        self.r_motor.weak_turn(-64, 360)

    def turn_right(self):
        self.l_motor.weak_turn(64, 360)

    def turn_left(self):
        self.r_motor.weak_turn(64, 360)

    def brake(self):
        self.l_motor.brake()
        self.r_motor.brake()


def main():
    rospy.init_node('car', log_level=rospy.INFO)

    car = Car()

    rospy.spin()

if __name__ == '__main__':
    main()
