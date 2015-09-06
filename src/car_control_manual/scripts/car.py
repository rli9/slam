#!/usr/bin/python
from __future__ import print_function
__author__ = 'flex'

import argparse

import rospy
from pyquark.arduino import *
from std_msgs.msg import String
import Queue


MOVE_MUTI =  170000
TURN_MUTI = 250000
# turn 90 degree: 250000
# move on setp 170000

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
            analogWrite(self.speed_pin, speed)

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
        self.que = Queue.Queue()
        self.actions = {u'forward': self.move_forward,
                        u'turnRight': self.turn_right,
                        u'turnLeft': self.turn_left,
                        u'setVideoState': self.take_photo,
                        u'doRepeat': self.doRepeat}
        self.control_sub = rospy.Subscriber('car_control_manual', String, callback=self.control_callback, queue_size=10)
        self.take_photo_pub = rospy.Publisher('car_control_manual/take_photo', String, queue_size=100)
        self.prev_object_region_x = None
        self.prev_object_region_y = None

    def control_callback(self, data):
        # you can not turn and move in the same time now...
        print("Received From Host %s, Type: %s" %(data, type(data)))
        # command, params = data.data.split(':',1)
        # params = eval(params)

        # self.actions[command](**params)
        self.main( eval( data.data)[0][2] )

    def take_photo(self,*args):
        self.take_photo_pub.publish(args[0]) # args[0] equals on/off
        print("Published Take photo %s" % args[0])

    def doRepeat(self, times, *args, **kwargs):
        print("times %s " % times)
        for i in range(times):
            print("In repeat %s === args:%s " % ( times, str(args)))
            for cmds in args:
                self.main(cmds)

    def stop(self, *args):
        self.right_motor.stop()
        self.left_motor.stop()

    def move(self, direction,*args, **kargs):
        if 'speed' not in kargs:
            kargs['speed'] = 70

        speed = int(kargs['speed'])
        kargs['distance'] = args[0]
        print("Move speed: %s" % speed)
        self.left_motor.run(direction, speed)
        self.right_motor.run(direction, speed)

        if 'distance' in kargs:
            ms = float(kargs['distance'])*MOVE_MUTI / speed
            print('Distance: %s , Delayed: %s' %(kargs['distance'], ms))
            delay(ms)
            print('Move done.')
            self.stop()
            delay(1000)

    move_forward = method_partial(move, LOW)
    move_backward = method_partial(move, HIGH)

    def turn(self, left_direction, right_direction,*args, **kargs):
        if 'speed' not in kargs:
            kargs['speed'] = dict(left=70, right=70)

        kargs['rotation'] = args[0]
        left_speed = int(kargs['speed']['left'])
        right_speed = int(kargs['speed']['right'])

        print("Turn left speed %s , Turn right speed %s" % (left_speed, right_speed))
        self.left_motor.run(left_direction, left_speed)
        self.right_motor.run(right_direction, right_speed)

        if 'rotation' in kargs:
            ms = float(kargs['rotation'])*(TURN_MUTI*1.0/90) / ((left_speed + right_speed) / 2)
            print('Rotation: %s , Delayed: %s' %(kargs['rotation'], ms))
            delay(ms)
            print('Turn Done.')
            self.stop()
            delay(1000)

    turn_left = method_partial(turn, LOW, HIGH)
    turn_right = method_partial(turn, HIGH, LOW)

    def cmd_parser(self, cmd):
        if cmd[0].endswith(':'):
            cmd[0] = cmd[0][:-1]
        try:
            print("Cmd: %s, params %s " % (cmd[0], cmd[1:]))
            self.actions[cmd[0]](*cmd[1:])
        except KeyError, e:
            print("Actions does not have key : %s" % cmd[0])

    def main(self,cmds):
        for cmd in cmds:
            self.cmd_parser(cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--light_pins', nargs=4, help='pins of lights system in sequence of head, tail, left turn and right turn')
    parser.add_argument('--turn_delay', type=int, help='delay between starting turn left/right and stopping')
    parser.add_argument('--min_turn_speed', type=int, help='speed of turn left/right')

    args = parser.parse_args()
    rospy.init_node('car', log_level=rospy.INFO)
    rospy.loginfo('__main__ %s' % args)

    car = Car( light_pins=args.light_pins, turn_delay=args.turn_delay, min_turn_speed=args.min_turn_speed)
    rospy.spin()

