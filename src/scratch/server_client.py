# -*- encoding: utf-8 -*-
from __future__ import print_function

__author__ = 'Simon Zheng'

"""Transfer command data from scratch to linux host
"""

import socket
from std_msgs.msg import String
import rospy

try:
    from pyquark.arduino import *
except:
    print("Module Not Found : pyquark.ardunio")


def check_format(func):

    def wrapped(*args, **kargs):
        self = args[0]
        # self defined command

        # command format : [[cmd, key1,value1,key2,value2...],[],[]]
        msg = eval(kargs['msg'])[-1][2]
        print("Received Msg %s" % msg)
        # the first two args is position in scratch
        # cmd_map = {}
        cmd_map = []
        for cmd in msg:
            if cmd[0] == 'procDef':
                continue
            action = cmd[1].split(' ')[0]
            params = {}
            for i in range(2, len(cmd)-1 ,2):
                if i >= len(cmd):
                    break
                print("i: %s, cmd %s, i+1: %s, cmd: %s"%(i,cmd[i], i+1, cmd[i+1]))
                params[cmd[i]] = cmd[i+1]

            # cmd_map[action] = params
            cmd_map.append( (action, params) )

        print("Get Cmd Map ", cmd_map)
        func(self, cmd_map)
        print('Cmd publish DONE')

    return wrapped


class Server(object):
    def __init__(self, host='', port=50007):
        self.host = host
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.host, self.port))
        self.conn = None  # received socket connection
        self.address = None  # bind socket address

        self.pub = rospy.Publisher('/car_control_manual', String, queue_size=100)
        rospy.init_node('car_control_manual', log_level=rospy.INFO)
        print("Publisher Ready.", rospy.is_shutdown())
        ret = self.pub.publish(String('The first Msg.'))
        print("Publish Ret  %s" % ret)

    def listen(self, maxinum=1, buffer=1024):
        '''
        data format [[97, 36, [[u'forward:', 10], [u'turnRight:', 15], [u'turnLeft:', 15]]]]
        :param maxinum: max num to listen
        :param buffer: receive msg buffer
        :return: None
        '''
        while True:
            self.s.listen(maxinum)
            self.conn, self.address = self.s.accept()
            print('Connected by', self.address)
            while True:
                data = self.conn.recv(buffer)
                if data:
                    print("Raw Data : ", data)
                    print(type(data))
                    self.pub.publish( data )
                    #self.publish_cmd(msg=data)
                    print("publish done.")
                self.conn.sendall("ok")

    @check_format
    def publish_cmd(self, cmd_dict):
        for key in cmd_dict:
            # self.pub.publish(key+":"+str(cmd_dict[key]))
            self.pub.publish(key[0]+":"+str(key[1]))
            # print("Published %s" % key+":"+str(cmd_dict[key]))
            print("published %s "% key[0]+":"+str(key[1]))

    def close(self):
        self.conn.close()


class Client(object):
    def __init__(self, remote_host, port):
        self.host = remote_host
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.s.connect((self.host, self.port))

    def send(self, msg, *args, **kwargs):
        self.s.sendall(msg)
        print("Send msg %s" % (msg))
        data = self.s.recv(1024)
        return data

    def close(self):
        self.s.close()
