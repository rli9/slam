#!/usr/bin/env python

import roslib
roslib.load_manifest('cherokey_car')

import rospy
import logging
from pygalileo.arduino import *
from std_msgs.msg import String

import argparse

def method_partial(func, *partial_args):
    return lambda self, *args, **kwargs: func(self, *(partial_args + args), **kwargs)

class Car(object):
    class Motor(object):
        def __init__(self, direction_pin, speed_pin):
            self.direction_pin = int(direction_pin)
            self.speed_pin = int(speed_pin)

            pinMode(self.direction_pin, OUTPUT)
            pinMode(self.speed_pin, OUTPUT)

            self.stop()

        def run(self, direction, speed):
            digitalWrite(self.direction_pin, direction)
            digitalWrite(self.speed_pin, HIGH)

        def stop(self):
            digitalWrite(self.direction_pin, LOW)
            digitalWrite(self.speed_pin, LOW)

    class Light(object):
        def __init__(self, pin):
            self.pin = int(pin)
            pinMode(self.pin, OUTPUT)
            self.off()

        def on(self):
            digitalWrite(self.pin, HIGH)

        def off(self):
            digitalWrite(self.pin, LOW)

    class EmptyLight(object):
        def on(self):
            return

        def off(self):
            return

    def __init__(self, light_pins):
        rospy.loginfo('Car.__init__')

        self.right_motor = Car.Motor(4, 5)
        self.left_motor = Car.Motor(7, 6)

        if light_pins is not None:
            self.lights = {"head": Car.Light(light_pins[0]), "tail": Car.Light(light_pins[1]), "left_turn": Car.Light(light_pins[2]), "right_turn": Car.Light(light_pins[3])}
        else:
            self.lights = {"head": Car.EmptyLight(), "tail": Car.EmptyLight(), "left_turn": Car.EmptyLight(), "right_turn": Car.EmptyLight()}

        self.sub = rospy.Subscriber('car_control', String, self.control_callback)

    def control_callback(self, data):
        action = ','.join(["'%s':'%s'" % (part.strip().split(' ')[0], part.strip().split(' ')[1]) for part in data.data.split(',')])

        rospy.loginfo('Car.control_callback %s' % action)

        action = eval("{%s}" % action)

        actions = {"w": self.move_forward, "s": self.move_backward, "a": self.turn_left, "d": self.turn_right}

        direction = action.pop('direction')
        actions[direction](**action)
        # delay(3000)
        self.stop()

    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()
        for (key, value) in self.lights.items():
            value.off()

    def move(self, direction, light, **args):
        self.lights[light].on()

        if 'speed' not in args:
            args["speed"] = {"left": 255, "right": 255}
        rospy.loginfo('Car.move(%s, %s, %s)' % (direction, light, args))

        speed = args["speed"]

        self.left_motor.run(direction, speed["left"])
        self.right_motor.run(direction, speed["right"])

        if "distance" in args:
            delay(int(args["distance"]) / ((speed["left"] + speed["right"]) / 2))

            self.stop()
            self.lights[light].off()

    move_forward = method_partial(move, HIGH, "head")
    move_backward = method_partial(move, LOW, "tail")

    def turn(self, left_direction, right_direction, light, **args):
        self.lights[light].on()

        if 'speed' not in args:
            args["speed"] = {"left": 255, "right": 255}
        rospy.loginfo('Car.move(%s, %s, %s, %s)' % (left_direction, right_direction, light, args))

        speed = args["speed"]

        self.left_motor.run(left_direction, speed["left"])
        self.right_motor.run(right_direction, speed["right"])

        if "rotation" in args:
            delay(int(args["rotation"]) / ((speed["left"] + speed["right"]) / 2))

            self.stop()
            self.lights[light].off()

    turn_left = method_partial(turn, LOW, HIGH, "left_turn")
    turn_right = method_partial(turn, HIGH, LOW, "right_turn")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--light_pins', nargs=4, help='pins of lights system in sequence of head, tail, left turn and right turn')

    args = parser.parse_args()

    log = logging.getLogger('pygalileo')
    log.setLevel(logging.INFO)

    rospy.init_node('car', log_level=rospy.INFO)
    rospy.loginfo('__main__ %s' % args)

    car = Car(args.light_pins)

    rospy.spin()
