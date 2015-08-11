# -*- encoding: utf-8 -*-
from __future__ import print_function

__author__ = 'Simon Zheng'

"""Transfer command data from scratch to linux host
"""

import socket


def check_format(func):
    cmds = ['forward:', 'turnRight:', 'turnLeft:']

    def wrapped(*args, **kargs):
        self = args[0]
        msg = eval(kargs['msg'])[0][2]
        print("Received Msg %s" % msg)
        # the first two args is position in scratch
        cmd_map = {}
        for cmd in msg:
            print("Cmd %s , Value %s" % (cmd[0], cmd[1]))
            if cmd[0] not in cmds:
                print("cmd error, we don't have", cmd)
                return 'cmd error'
            else:
                # remove the :
                cmd_map[cmd[0][:-1]] = cmd[1]
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

    def listen(self, maxinum=1, buffer=1024):
        '''
        data format [[97, 36, [[u'forward:', 10], [u'turnRight:', 15], [u'turnLeft:', 15]]]]
        :param maxinum: max num to listen
        :param buffer: receive msg buffer
        :return: None
        '''
        self.s.listen(maxinum)
        self.conn, self.address = self.s.accept()
        print('Connected by', self.address)
        while True:
            data = self.conn.recv(buffer)
            if data:
                self.publish_cmd(msg=data)
            self.conn.sendall("ok")

    @check_format
    def publish_cmd(self, cmd_dict):
        print("In function %s, %s" % (self.publish_cmd.__name__, cmd_dict))

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


if __name__ == "__main__":
    import os

    if os.getenv('ENV', None) == 'client' or not os.getenv('ENV', None):
        host = '192.168.1.219'
        port = 50007

        cli = Client(host, port)
        cli.connect()
        res = cli.send("Test msg socket")
        if res == "ok":
            print("test ok")

        cli.close()
    elif os.getenv('ENV', None) == 'server':
        ser = Server()
        ser.listen()
        ser.close()
    else:
        print("Please set your EVN var of system")
