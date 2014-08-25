#!/usr/bin/env python

import rospy
import wx
import types
from std_msgs.msg import String


class CarControl(object):
    def __init__(self):
        rospy.init_node("car_control", anonymous=True, log_level=rospy.INFO)

        self.pub = rospy.Publisher('car_control', String)

        for direction in ['forward', 'backward', 'right', 'left']:
            self._define_move_attr(direction)

    def status(self):
        rospy.loginfo("data: %f" % (self.joint_state))

    def _define_move_attr(self, direction):
        def _move(self):
            self.pub.publish('move_%s' % direction)

        _move.__name__ = 'move_%s' % direction

        setattr(self, 'move_%s' % direction, types.MethodType(_move, self))


class CarControlUI(object):
    def __init__(self):
        self.car_control = CarControl()

        self.frame = wx.Frame(None, title='car control', size=(820, 500))

        self.forward_button = wx.Button(self.frame, label="F", pos=(710, 380), size=(50, 50))
        self.backward_button = wx.Button(self.frame, label="B", pos=(710, 430), size=(50, 50))
        self.right_button = wx.Button(self.frame, label="R", pos=(760, 430), size=(50, 50))
        self.left_button = wx.Button(self.frame, label="L", pos=(660, 430), size=(50, 50))

        for direction in ['forward', 'backward', 'right', 'left']:
            self._bind_on_button_clicked(direction)

        self.frame.Show()

    def _bind_on_button_clicked(self, direction):
        def _on_button_clicked(self, event):
            getattr(self.car_control, 'move_%s' % direction)()

        _on_button_clicked.__name__ = 'on_button_clicked_%s' % direction

        getattr(self, '%s_button' % direction).Bind(wx.EVT_BUTTON, types.MethodType(_on_button_clicked, self))


def main():
    app = wx.App()
    car_control_ui = CarControlUI()
    app.MainLoop()


if __name__ == '__main__':
    main()
