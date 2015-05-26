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
            analogWrite(self.speed_pin, speed)

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

        actions = {"forward": self.move_forward, "backward": self.move_backward, "left": self.turn_left, "right": self.turn_right, "follow": self.follow}

        direction = action.pop('direction')
        actions[direction](**action)
        # delay(3000)
        self.stop()

    def follow(self, **args):
        if 'width' not in args:
            args['width'] = 1280
        if 'height' not in args:
            args['height'] = 720

        width = int(args['width'])
        height = int(args['height'])

        half_width = width / 2
        quater_width = width / 4

        half_height = height / 2
        quater_height = height / 4

        tolerant_error_width = width / 8
        tolerant_error_height = height / 8

        object_region_x = ((int(args['object_x']) - half_width) + quater_width - tolerant_error_width) / quater_width
        object_region_y = ((int(args['object_y']) - half_height) + quater_height - tolerant_error_height) / quater_height

        rospy.loginfo('Car.follow(object_region_x=%d, object_region_y=%d)' % (object_region_x, object_region_y))

        if object_region_x > 0:
            self.turn_right(rotation=50000 * object_region_x)
        elif object_region_x < 0:
            self.turn_left(rotation=-50000 * object_region_x)

        if object_region_y > 0:
            self.move_backward(distance=50000 * object_region_y)
        elif object_region_y < 0:
            self.move_forward(distance=-50000 * object_region_y)

    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()
        for (key, value) in self.lights.items():
            value.off()

    def move(self, direction, light, **args):
        self.lights[light].on()

        if 'speed' not in args:
            args["speed"] = 100
        rospy.loginfo('Car.move(%s, %s, %s)' % (direction, light, args))

        speed = int(args["speed"])

        self.left_motor.run(direction, speed)
        self.right_motor.run(direction, speed)

        if "distance" in args:
            ms = int(args["distance"]) / speed

            rospy.loginfo('Car.move(%d)' % ms)
            delay(ms)

            self.stop()
            self.lights[light].off()

    move_forward = method_partial(move, LOW, "head")
    move_backward = method_partial(move, HIGH, "tail")

    def turn(self, left_direction, right_direction, light, **args):
        self.lights[light].on()

        if 'speed' not in args:
            args["speed"] = {"left": 100, "right": 100}
        rospy.loginfo('Car.move(%s, %s, %s, %s)' % (left_direction, right_direction, light, args))

        left_speed = int(args["speed"]["left"])
        right_speed = int(args["speed"]["right"])

        self.left_motor.run(left_direction, left_speed)
        self.right_motor.run(right_direction, right_speed)

        if "rotation" in args:
            ms = int(args["rotation"]) / ((left_speed + right_speed) / 2)

            rospy.loginfo('Car.move(%d)' % ms)
            delay(ms)

            self.stop()
            self.lights[light].off()

    turn_left = method_partial(turn, LOW, HIGH, "left_turn")
    turn_right = method_partial(turn, HIGH, LOW, "right_turn")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--light_pins', nargs=4, help='pins of lights system in sequence of head, tail, left turn and right turn')

    args = parser.parse_args()

    # log = logging.getLogger('pygalileo')
    # log.setLevel(logging.INFO)

    rospy.init_node('car', log_level=rospy.INFO)
    rospy.loginfo('__main__ %s' % args)

    car = Car(args.light_pins)

    rospy.spin()
