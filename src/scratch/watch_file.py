from __future__ import print_function

"""Watch File generated by Scratch
    1. save Scratch file *.sb2 into the same directory or specify with path
    2. change name *.sb2 to *.zip, on linux this is unnessary
    3. unzip *.zip file and read json data from project.json
"""

import time, os, zipfile, json
import watchdog
from watchdog.observers import Observer
import platform

system = platform.system()

class MyFileMonitor(watchdog.events.FileSystemEventHandler):
    def __init__(self, suffix, callback):
        super(MyFileMonitor, self).__init__()
        self.callback = callback
        if suffix.startswith('.'):
            self.suffix = suffix[1:]
        else:
            self.suffix = suffix

    def on_created(self, event):
        super(MyFileMonitor, self).on_created(event)
        n_suffix = event.src_path.split('.')[-1]
        if not event.is_directory and n_suffix == self.suffix:
            # when detected file created which we need , use callback to deal with
            self.callback(event.src_path)


class WatchFile(object):
    def __init__(self, send_msg_func, *args, **kargs):
        self.path = kargs['path'] if kargs.has_key('path') else '.'
        self.suffix = kargs['suffix'] if kargs.has_key('suffix') else '*'  # star represent any file
        self.observer = Observer()
        self.event_handler = MyFileMonitor(self.suffix, callback=self.get_data)
        self.send_msg_func = send_msg_func
        self.filename = self.zip_filename = ''

    def run(self):
        self.observer.schedule(self.event_handler, self.path, recursive=True)
        self.observer.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.observer.stop()
        self.observer.join()

    def get_data(self, filename):
        data = self._unpack(filename)
        data = str(data)
        print(data, type(data))
        self.send_msg_func(data)

    def _unpack(self, filename):
        # first rename suffix to zip file
        # may not work on linux
        if system == 'Windows':
            filename = filename[2:] if filename.startswith('.\\') else filename
            filename = filename.lstrip()
            new_name = filename.split('.')[0] + '.zip'
            new_name = new_name[1:] if new_name.startswith('\\') else new_name
        elif system == 'Linux':
            new_name = filename

        print('Old name:', filename, ' New name:', new_name)

        self.filename = filename
        self.zip_filename = new_name
        # waiting for operating sys create the file
        time.sleep(3)
        os.rename(filename, new_name)
        zip_file = zipfile.ZipFile(new_name, 'r')
        json_data = ""
        for name in zip_file.namelist():
            if name == "project.json":
                file = zip_file.open(name, 'r')
                json_data = "".join(file.readlines())
        # change filename back to .sb2
        if new_name.endswith('.zip'):
            os.rename(new_name, filename)

        return self.get_cmd(json_data)

    def get_cmd(self, json_data):
        jsonfy_data = json.loads(json_data)
        child = jsonfy_data['children'][0]
        scripts = child['scripts']
        return scripts


def send_msg_test_func(msg):
    print("send msg ok %s" % msg)


if __name__ == "__main__":
    wd = WatchFile(send_msg_test_func, suffix=".sb2")
    wd.run()
