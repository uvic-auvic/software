import PySpin

# Created for AUVIC by Oscar Morrison
# Encapsulates PySpin API for easy Spinnaker streaming in OpenCV

class SpinCamera:
    # Initilizes camera pointer with first spinnaker device availible
    def __init__(self):
        self.isStreaming = False
        try:
            self.system = PySpin.System.GetInstance()
            self.cameraList = self.system.GetCameras()
            if(len(self.cameraList) == 0):
                print("No cameras found!")
                raise Exception()
            else:
                self.camera = self.cameraList[0]
        
        except PySpin.SpinnakerException as se:
            print("Spinnaker exception during enumeration")

    # Loads node map and starts streaming camera
    # After starting streaming frames immediately start filling frmae buffer
    def startCamera(self):
        try:
            self.tdDevice = self.camera.GetTLDeviceNodeMap()
            self.camera.Init()
            self.nodeMap = self.camera.GetNodeMap()

            self.nodeAcquisitionMode = PySpin.CEnumerationPtr(self.nodeMap.GetNode('AcquisitionMode'))
            nodeAcquisitionModeContinuous = self.nodeAcquisitionMode.GetEntryByName('Continuous')
            acquisitionModeContinuous = nodeAcquisitionModeContinuous.GetValue()
            self.nodeAcquisitionMode.SetIntValue(acquisitionModeContinuous)
        
        except PySpin.SpinnakerException as se:
            print("Spinnaker exception during camera start")
            return False

        self.camera.BeginAcquisition()
        print("Starting image aquistion")

        deviceNumberNode = PySpin.CStringPtr(self.tdDevice.GetNode('DeviceSerialNumber'))
        if PySpin.IsAvailable(deviceNumberNode) and PySpin.IsReadable(deviceNumberNode):
            deviceNumber = deviceNumberNode.GetValue()

        return True

    # Grabs a numpy ndarray from buffer (p_width x p_height x color_channels)
    def grabImageArray(self):
        imageResult = self.camera.GetNextImage(1000)
        
        if imageResult.IsIncomplete():
            print("Grabbed Incomplete Image")
        
        imageConverted = imageResult.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR)

        imageResult.Release()

        return imageConverted.GetNDArray()

    # Stops streaming camera
    def stopCamera(self):
        self.camera.EndAcquisition()
        self.camera.DeInit()
        print("Image aquisition stopped")

    # Releases camera
    def tearDown(self):
        del self.camera
        self.cameraList.Clear()
        self.system.ReleaseInstance()


