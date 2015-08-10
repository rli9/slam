# -*- coding: utf-8 -*-
from __future__ import print_function

__author__ = 'Simon Zheng'

import connect_to_host as connect
import watch_file as wf

Host = "192.168.1.219"
Port = 50007

if __name__ == "__main__":
    cli = connect.Client(remote_host=Host, port=Port)
    cli.connect()
    wd = wf.WatchFile(suffix=".sb2", send_msg_func=cli.send)
    wd.run()
