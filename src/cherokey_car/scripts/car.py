#!/usr/bin/env python

import roslib
roslib.load_manifest('cherokey_car')

import rospy
import logging
from pygalileo.arduino import *
from std_msgs.msg import String

class Car(object):
    class Motor(object):
        def __init__(self, direction_pin, speed_pin):
            self.direction_pin = direction_pin
            self.speed_pin = speed_pin

            pinMode(self.direction_pin, OUTPUT)
            pinMode(self.speed_pin, OUTPUT)

            self.stop()

        def run(self, direction, speed):
            analogyWrite(self.speed_pin, speed)
            digitalWrite(self.direction_pin, direction)

        def stop(self):
            digitalWrite(self.direction_pin, LOW)
            digitalWrite(self.speed_pin, LOW)

    def __init__(self):
        self.right_motor = Car.Motor(4, 5)
        self.left_motor = Car.Motor(7, 6)

        self.sub = rospy.Subscriber('car_control', String, self.control_callback)
        rospy.loginfo('Car.__init__')

    def control_callback(self, data):
        self.move_forward(100, 100)
        delay(5000)
        self.stop()

    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()

    def move_forward(self, left_speed, right_speed):
        self.left_motor.run(HIGH, left_speed)
        self.right_motor.run(HIGH, right_speed)

def main():
    log = logging.getLogger('pygalileo')
    log.setLevel(logging.INFO)

    rospy.init_node('car', log_level=rospy.INFO)

    car = Car()

    rospy.spin()

if __name__ == '__main__':
    main()
