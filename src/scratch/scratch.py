# -*- coding: utf-8 -*-
from __future__ import print_function

__author__ = 'Simon Zheng'

import server_client as connect
import watch_file as wf
import socket
import os
import rospy
Host = os.environ.get('SCRATCH_HOST') or '192.168.1.219'
Port = os.environ.get('SCRATCH_PORT') or 50007

print("Host %s, PORT %s" %(Host, Port))

def client_main():
    cli = connect.Client(remote_host=Host, port=Port)
    try:
        cli.connect()
        wd = wf.WatchFile(suffix=".sb2", send_msg_func=cli.send)
        wd.run()
    except socket.error, e:
        print("Error ", e)
        print("Just for test")
        wd = wf.WatchFile(suffix=".sb2", send_msg_func=wf.send_msg_test_func)
        wd.run()


def server_main():
    ser = connect.Server()
    ser.listen()


if __name__ == "__main__":
    import sys

    if sys.argv[1] == 'client':
        client_main()
    elif sys.argv[1] == 'server':
        server_main()
