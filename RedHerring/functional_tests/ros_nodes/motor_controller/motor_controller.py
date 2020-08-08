from emulator import DeviceEmulator
# import imp
import os

class motor_controller():
    def __init__(self):
        self.dev = DeviceEmulator()
        pass

    def getParameters(self):
        return {
            'port': self.dev.getPort()
        }

    def start(self):
        i = 0
        while True:
            data = self.dev.read().strip()
            if data.startswith('M'):
                self.move(data)
            elif data.startswith('STP'):
                self.stop(data)
            elif data.startswith('SM'):
                self.stop(data)
            elif data.startswith('RV'):
                self.revolutions(data)
            elif data.startswith('PW'):
                self.pwm(data)
            elif data.startswith('RID'):
                self.rid(data)
            elif data.startswith('TMP'):
                self.temperature(data)
            else:
                pass

    def move(self, cmd):
        pass

    def stop(self, cmd):
        pass

    def revolutions(self, cmd):
        pass

    def pwm(self, data):
        pass

    def calibrate(self, cmd):
        pass

    def rid(self):
        pass

    def temperature(self, data):
        pass


if __name__ == '__main__':
    a = motor_controller()
    a.start()