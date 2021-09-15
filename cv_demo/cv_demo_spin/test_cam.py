# Quick test code
from spin_stream import SpinCamera

# Get camera instance
cam = SpinCamera()

# Set up camera trigger
cam.configureAsTriggered()

# Start Aquisition
cam.startCamera()

# Trigger camera frame aquisition
cam.triggerCamera()

# Grab image off frame buffer as np array
cam.grabImageArray()

# Reset triggered camera to non-triggered camera
cam.resetTriggeredCamera()

# Stop Aquisition
cam.stopCamera()

# Destroy camera instance
cam.tearDown()