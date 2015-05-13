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
            digitalWrite(self.speed_pin, HIGH)
            digitalWrite(self.direction_pin, direction)

        def stop(self):
            digitalWrite(self.direction_pin, LOW)
            digitalWrite(self.speed_pin, LOW)

    def __init__(self):
        rospy.loginfo('Car.__init__')
        
        self.right_motor = Car.Motor(4, 5)
        self.left_motor = Car.Motor(7, 6)

        self.sub = rospy.Subscriber('car_control', String, self.control_callback)

    def control_callback(self, data):
        rospy.loginfo('Car.control_callback %s' % data.data)
        
        actions = {"w": self.move_forward, "s": self.move_backward, "a": self.turn_left, "d": self.turn_right}
        
        actions[data.data](100, 100)
        delay(3000)
        self.stop()

    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()

    def move_forward(self, left_speed, right_speed):
        self.left_motor.run(HIGH, left_speed)
        self.right_motor.run(HIGH, right_speed)
        
    def move_backward(self, left_speed, right_speed):
        self.left_motor.run(LOW, left_speed)
        self.right_motor.run(LOW, right_speed)
        
    def turn_left(self, left_speed, right_speed):
        self.left_motor.run(LOW, left_speed)
        self.right_motor.run(HIGH, right_speed)
        
    def turn_right(self, left_speed, right_speed):
        self.left_motor.run(HIGH, left_speed)
        self.right_motor.run(LOW, right_speed)        
        
def main():
    log = logging.getLogger('pygalileo')
    log.setLevel(logging.INFO)

    rospy.init_node('car', log_level=rospy.INFO)

    car = Car()

    rospy.spin()

if __name__ == '__main__':
    main()
