#!/usr/bin/python
from __future__ import print_function
import wx
import rospy
import Queue
import cv2

from car_control_manual.msg import CarControlMsg
from std_msgs.msg import String
import numpy as np
WXKEY_UP = 0x1
WXKEY_DOWN = 0x2
WXKEY_LEFT = 0x4
WXKEY_RIGHT = 0x8

KEY_MAPS = {
    wx.WXK_LEFT: WXKEY_LEFT,
    wx.WXK_RIGHT: WXKEY_RIGHT,
    wx.WXK_DOWN: WXKEY_DOWN,
    wx.WXK_UP: WXKEY_UP
}


class DisplayPanel(wx.Panel):
    def __init__(self, parent, fps=15):
        wx.Panel.__init__(self, parent, size=(640,400))
        #self.capture = capture
        #ret, frame = self.capture.read()

        # keyboard bind
        self.down_keys = 0
        self.pub = rospy.Publisher('car_control_manual', String, queue_size = 10)
        self.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        self.Bind(wx.EVT_KEY_UP, self.on_key_up)

        # Image bind
        self.img_sub = rospy.Subscriber('image_publisher', String, self.img_callback, queue_size = 1 )
        self.cap_que = Queue.Queue(maxsize=10)

        self.bmp = wx.BitmapFromBuffer(640, 480, self.cap_que.get())

        self.Bind(wx.EVT_PAINT, self.on_paint)
        #height, width = frame.shape[:2]
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        self.timer = wx.Timer(self)
        self.timer.Start(200.0/fps)
        self.Bind(wx.EVT_TIMER, self.next_frame)

    def img_callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        try:
            if self.cap_que.full():
                return
            self.cap_que.put(image_np)
            #self.next_frame(image_np)
            #cv2.waitKey(3)
        except BaseException, e:
            print(e)


    def on_key_down(self, event):
        key_code = event.GetKeyCode()

        for k,v in KEY_MAPS.iteritems():
            if key_code == k:
                self.notify_velocity_change(self.down_keys | v)

    def on_key_up(self, event):
        key_code = event.GetKeyCode()

        for k,v in KEY_MAPS.iteritems():
            if key_code == k:
                self.notify_velocity_change(self.down_keys & ~v)

    def notify_velocity_change(self, down_keys):
        self.down_keys = down_keys
        car_control_msg = CarControlMsg()
        car_control_msg.x = car_control_msg.y = 0
        if(self.down_keys & WXKEY_UP) != 0:
            car_control_msg.y +=1
        if(self.down_keys & WXKEY_DOWN) != 0:
            car_control_msg.y -=1
        if(self.down_keys & WXKEY_RIGHT) != 0:
            car_control_msg.x +=1
        if(self.down_keys & WXKEY_LEFT) != 0:
            car_control_msg.x -=1

        self.pub.publish(car_control_msg)

    def on_paint(self, evt):
        dc = wx.BufferedPaintDC(self)
        if hasattr(self, 'bmp'):
            dc.DrawBitmap(self.bmp, 0, 0)

    def next_frame(self,frame):
        #self.on_paint(frame)
        frame = self.cap_que.get()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.bmp.CopyFromBuffer( frame )
        self.Refresh()


class CarControlManualUI(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, wx.ID_ANY ,
                          title="Car control manual",
                          size = (640, 480),
                          style=wx.DEFAULT_FRAME_STYLE & (~(wx.RESIZE_BORDER | wx.MAXIMIZE_BOX)))
        self.Center()


def main():
    rospy.init_node('car_control_manual', log_level = rospy.INFO)
    app = wx.App()
    ui = CarControlManualUI()
    # Display Panel
    #capture = cv2.VideoCapture(1)
    DisplayPanel(ui)
    # keyBoard Panel
    #keyboardPanel(ui)
    ui.Show()
    app.MainLoop()


if __name__ == "__main__":
    main()