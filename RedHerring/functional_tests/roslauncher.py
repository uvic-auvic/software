from datetime import datetime
import subprocess as sub
import os

class LaunchfileCreator(object):
    def __init__(self, parameters):
        self.parameters = parameters
        self.getFileContents()

    def getFileContents(self, launchfile_name='template_launch_file.launch'):
        self.fpath = os.path.dirname(__file__)
        full_path = os.path.join(self.fpath, launchfile_name)
        with open(full_path) as f:
            self.raw_contents = f.read()

    def __enter__(self):
        launchfile = self.raw_contents.format(**self.parameters)
        filename = datetime.now().strftime('%Y-%m-%d_%H-%M-%S-temp.launch')
        self.full_file_path = os.path.join(self.fpath, filename)
        self.file = open(self.full_file_path, 'w')
        self.file.write(launchfile)
        self.file.close()
        return self

    def getName(self):
        return self.full_file_path

    def __exit__(self, exc_type, exc_value, traceback):
        os.remove(self.getName())


class Roslauncher(object):
    ros = 'roscore'
    def __init__(self, parameters):
        with LaunchfileCreator(parameters) as f:
            self.args = [self.ros, f.getName()]
            self.proc = sub.Popen(self.args, stdout=sub.PIPE, stderr=sub.PIPE)


if __name__ == '__main__':
    pass
