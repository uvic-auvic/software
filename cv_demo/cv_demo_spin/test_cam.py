# Quick test code
from spin_stream import SpinCamera
cam = SpinCamera()
cam.startCamera()
cam.grabImageArray()
cam.stopCamera()
cam.tearDown()