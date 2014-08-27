#!/usr/bin/env python

import rospy
import wx
import types
from nxt_car.msg import CarControl


WXKEY_UP = 0x1
WXKEY_DOWN = 0x2
WXKEY_LEFT = 0x4
WXKEY_RIGHT = 0x8

KEY_MAPS = {wx.WXK_LEFT: WXKEY_LEFT, wx.WXK_RIGHT: WXKEY_RIGHT,
            wx.WXK_UP: WXKEY_UP, wx.WXK_DOWN: WXKEY_DOWN}


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

        self.pub = rospy.Publisher('car_control', CarControl)
        self.down_keys = 0

    def on_key_down(self, event):
        key_code = event.GetKeyCode()

        for k, v in KEY_MAPS.iteritems():
            if key_code == k:
                self.notify_velocity_change(self.down_keys | v)

    def on_key_up(self, event):
        key_code = event.GetKeyCode()

        for k, v in KEY_MAPS.iteritems():
            if key_code == k:
                self.notify_velocity_change(self.down_keys & ~v)

    def notify_velocity_change(self, down_keys):
        self.down_keys = down_keys

        car_control_msg = CarControl()
        car_control_msg.velocity_x = car_control_msg.velocity_y = 0

        if (self.down_keys & WXKEY_UP) != 0:
            car_control_msg.velocity_y += 1

        if (self.down_keys & WXKEY_DOWN) != 0:
            car_control_msg.velocity_y -= 1

        if (self.down_keys & WXKEY_RIGHT) != 0:
            car_control_msg.velocity_x += 1

        if (self.down_keys & WXKEY_LEFT) != 0:
            car_control_msg.velocity_x -= 1

        self.pub.publish(car_control_msg)


def main():
    rospy.init_node('car_control', log_level=rospy.INFO)

    app = wx.App()
    car_control_ui = CarControlUI()
    app.MainLoop()

if __name__ == '__main__':
    main()
