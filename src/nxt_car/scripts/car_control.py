#!/usr/bin/env python

import rospy
import wx
import types
from std_msgs.msg import String


class CarControl(object):
    def __init__(self):
        rospy.init_node("car_control", anonymous=True, log_level=rospy.INFO)

        self.pub = rospy.Publisher('car_control', String)

        for action in ['move_forward', 'move_backward', 'turn_right', 'turn_left', 'brake']:
            self._define_action_attr(action)

    def _define_action_attr(self, action):
        def _action(self):
            self.pub.publish(action)
        _action.__name__ = action

        setattr(self, action, types.MethodType(_action, self))


class CarControlUI(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, -1,
                          title='car control',
                          size=(640, 480),
                          style=wx.DEFAULT_FRAME_STYLE & ~(wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))

        self.panel = wx.Panel(self, -1)
        self.panel.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        self.panel.Bind(wx.EVT_KEY_UP, self.on_key_up)

        self.Center()
        self.Show(True)

        self.car_control = CarControl()
        self.curr_key_code = None

    def on_key_down(self, event):
        key_code = event.GetKeyCode()
        self.SetTitle("%d" % key_code)

        for k, v in {wx.WXK_LEFT: 'turn_left', wx.WXK_RIGHT: 'turn_right',
                     wx.WXK_UP: 'move_forward', wx.WXK_DOWN: 'move_backward'}.iteritems():
            if key_code == k:
                if key_code != self.curr_key_code:
                    self.car_control.brake()

                self.curr_key_code = key_code
                getattr(self.car_control, v)()

        rospy.loginfo("on_key_down %d" % key_code)

    def on_key_up(self, event):
        key_code = event.GetKeyCode()

        if self.curr_key_code == key_code:
            self.car_control.brake()

        rospy.loginfo("on_key_up %d" % key_code)


def main():
    app = wx.App()
    car_control_ui = CarControlUI()
    app.MainLoop()


if __name__ == '__main__':
    main()
