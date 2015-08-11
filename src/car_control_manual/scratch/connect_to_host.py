# -*- encoding: utf-8 -*-
from __future__ import print_function

__author__ = 'Simon Zheng'

"""Transfer command data from scratch to linux host
"""

import socket


class Server(object):
    def __init__(self, host='', port=50007):
        self.host = host
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.host, self.port))
        self.conn = None  # received socket connection
        self.address = None  # bind socket address

    def listen(self, maxinum=1, buffer=1024):
        self.s.listen(maxinum)
        self.conn, self.address = self.s.accept()
        print('Connected by', self.address)
        while True:
            data = self.conn.recv(buffer)
            if not data: break
            self.conn.sendall("ok")

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
