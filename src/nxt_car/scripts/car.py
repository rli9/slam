#!/usr/bin/env python

import roslib
roslib.load_manifest('nxt_car')

import rospy
import nxt.locator
from nxt.motor import PORT_A, PORT_B
import nxt.motor
from nxt_car.msg import CarControl


class Car(object):
    def __init__(self):
        nxt.motor.Motor.debug = 1

        self.brick = nxt.locator.find_one_brick()

        self.r_motor = nxt.motor.Motor(self.brick, PORT_A)
        self.l_motor = nxt.motor.Motor(self.brick, PORT_B)

        self.sub = rospy.Subscriber('car_control', CarControl, self.control_callback)

    def control_callback(self, data):
        if data.velocity_x == 0 and data.velocity_y == 0:
            self.brake()
        else:
            action = {'l_motor': [0, 0], 'r_motor': [0, 0]}

            if data.velocity_y > 0:
                action.update(self.move_forward_action())
            elif data.velocity_y < 0:
                action.update(self.move_backward_action())

            if data.velocity_x > 0:
                action.update(self.turn_right_action())
            elif data.velocity_x < 0:
                action.update(self.turn_left_action())

            rospy.loginfo('control_callback(%s)' % action)

            self.l_motor.weak_turn(action['l_motor'][0], action['l_motor'][1])
            self.r_motor.weak_turn(action['r_motor'][0], action['r_motor'][1])

    def move_forward_action(self):
        return {'l_motor': [64, 360], 'r_motor': [64, 360]}

    def move_backward_action(self):
        return {'l_motor': [-64, 360], 'r_motor': [-64, 360]}

    def turn_right_action(self):
        return {'l_motor': [128, 720]}

    def turn_left_action(self):
        return {'r_motor': [128, 720]}

    def brake(self):
        self.l_motor.brake()
        self.r_motor.brake()


def main():
    rospy.init_node('car', log_level=rospy.INFO)

    car = Car()

    rospy.spin()

if __name__ == '__main__':
    main()
