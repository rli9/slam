#!/usr/bin/python
__author__ = 'flex'

import argparse

import rospy
from pyquark.arduino import *
from std_msgs.msg import String


def method_partial(func, *partial_args):
    return lambda self, *args, **kwargs: func(self, *(partial_args + args), **kwargs)


'''
command example : [[97, 36, [[u'forward:', 10], [u'turnRight:', 15], [u'turnLeft:', 15]]]]
'''


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
            digitalWrite(self.speed_pin, speed)

        def stop(self):
            digitalWrite(self.direction_pin, LOW)
            digitalWrite(self.speed_pin, LOW)

    class Light(object):
        def __init__(self, pin):
            self.pin = int(pin)
            pinMode(self.pin, OUTPUT)

        def on(self):
            digitalWrite(self.pin, HIGH)

        def off(self):
            digitalWrite(self.pin, LOW)

    DEFAULT_TURN_DELAY = 200
    MAX_SPEED = 100
    DEFAULT_MIN_LEFT_RIGHT_SPEED = 80
    MIN_FORWARD_BACKWARD_SPEED = 70

    def __init__(self, light_pins, **configs):
        self.right_motor = Car.Motor(4, 5)
        self.left_motor = Car.Motor(7, 6)

        self.configs = dict(turn_delay=self.DEFAULT_TURN_DELAY, min_turn_speed=self.DEFAULT_MIN_LEFT_RIGHT_SPEED)

        for key in self.configs.keys():
            self.configs[key] = configs[key] or self.configs[key]

        self.sub = rospy.Subscriber('car_control_manual', String, callback=self.control_callback, queue_size=10)

        self.prev_object_region_x = None
        self.prev_object_region_y = None

    def control_callback(self, data):
        # direct_x : -1 :left , 1: right
        # direct_y : -1 :back , 1: forward
        # you can not turn and move in the same time now...
        action = {u'forward': self.move_forward, u'turnRight': self.turn_right, u'turnLeft': self.turn_left}
        # Get action from scratch by using
        actions = eval(data)

        direction = actions.pop('direction')
        action[direction](**actions)

        # direct_x = data.x
        # direct_y = data.y
        #
        # if direct_x == 1:
        #     self.turn_right()
        # elif direct_x == -1:
        #     self.turn_left()
        # elif direct_y == 1:
        #     self.move_forward(speed=10)
        # elif direct_y == -1:
        #     self.move_backward(speed=10)
        # else:
        #     self.stop()

    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()

    def move(self, direction, **kargs):
        if 'speed' not in kargs:
            kargs['speed'] = 100

        speed = int(kargs['speed'])
        self.left_motor.run(direction, speed)
        self.right_motor.run(direction, speed)

        if 'distance' in kargs:
            ms = float(kargs['distance']) / speed
            delay(ms)
            self.stop()

    move_forward = method_partial(move, LOW)
    move_backward = method_partial(move, HIGH)

    def turn(self, left_direction, right_direction, **kargs):
        if 'speed' not in kargs:
            kargs['speed'] = dict(left=100, right=100)

        left_speed = int(kargs['speed']['left'])
        right_speed = int(kargs['speed']['right'])

        self.left_motor.run(left_direction, left_speed)
        self.right_motor.run(right_direction, right_speed)

        if 'rotation' in kargs:
            ms = int(kargs['rotation']) / ((left_speed + right_speed) / 2)
            delay(ms)
            self.stop()

    turn_left = method_partial(turn, LOW, HIGH)
    turn_right = method_partial(turn, HIGH, LOW)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # parser.add_argument('--light_pins', nargs=4, help='pins of lights system in sequence of head, tail, left turn and right turn')
    parser.add_argument('--turn_delay', type=int, help='delay between starting turn left/right and stopping')
    parser.add_argument('--min_turn_speed', type=int, help='speed of turn left/right')

    args = parser.parse_args()
    rospy.init_node('car', log_level=rospy.INFO)
    rospy.loginfo('__main__ %s' % args)

    car = Car(args.light_pins, turn_delay=args.turn_delay, min_turn_speed=args.min_turn_speed)
    rospy.spin()
