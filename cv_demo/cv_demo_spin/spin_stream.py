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

            nodeAcquisitionMode = PySpin.CEnumerationPtr(self.nodeMap.GetNode('AcquisitionMode'))
            nodeAcquisitionModeContinuous = nodeAcquisitionMode.GetEntryByName('Continuous')
            acquisitionModeContinuous = nodeAcquisitionModeContinuous.GetValue()
            nodeAcquisitionMode.SetIntValue(acquisitionModeContinuous)

            gainMode = PySpin.CEnumerationPtr(self.nodeMap.GetNode('GainAuto'))
            gainModeOnce = gainMode.GetEntryByName('Once')
            gainMode.SetIntValue(gainModeOnce.GetValue())
        
        except PySpin.SpinnakerException as se:
            print("Spinnaker exception during camera start")
            return False

        self.camera.BeginAcquisition()
        print("Starting image aquistion")

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

    # Configures camera with software trigger
    def configureAsTriggered(self):

        ## TODO make proper exceptions ##

        # Safe (although long winded) code style similar to PySpin trigger example
        try:
            self.tdDevice = self.camera.GetTLDeviceNodeMap()
            self.camera.Init()
            self.nodeMap = self.camera.GetNodeMap()

            triggerMode = PySpin.CEnumerationPtr(self.nodeMap.GetNode('TriggerMode'))
            if not PySpin.IsAvailable(triggerMode) or not PySpin.IsReadable(triggerMode):
                print("Unable to disable pre-extisting tigger mode Exception")
                raise Exception()
            
            triggerModeOff = triggerMode.GetEntryByName('Off')
            if not PySpin.IsAvailable(triggerModeOff) or not PySpin.IsReadable(triggerModeOff):
                print("Unable to get trigger selection. Exception")
                raise Exception()

            triggerMode.SetIntValue(triggerModeOff.GetValue())

            triggerSource = PySpin.CEnumerationPtr(self.nodeMap.GetNode('TriggerSource'))
            if not PySpin.IsAvailable(triggerSource) or not PySpin.IsWritable(triggerSource):
                print("Unable to get trigger source. Exception")
                raise Exception()

            triggerSourceSoftware = triggerSource.GetEntryByName("Software")
            if not PySpin.IsAvailable(triggerSourceSoftware) or not PySpin.IsReadable(triggerSourceSoftware):
                print("Unable to set trigger source. Exception")
                raise Exception()

            triggerSource.SetIntValue(triggerSourceSoftware.GetValue())

            triggerModeOn = triggerMode.GetEntryByName('On')
            if not PySpin.IsAvailable(triggerModeOn) or not PySpin.IsReadable(triggerModeOn):
                print("Unable to enable trigger")
                raise Exception()

            triggerMode.SetIntValue(triggerModeOn.GetValue())
            print("Spin camera trigger set")
        
        except PySpin.SpinnakerException as se:
            Print("Spinnaker error during trigger setup")
            return False

        return True

    # Trigers camera to send image to image buffer
    def triggerCamera(self):
        try:
            triggerCommand = PySpin.CCommandPtr(self.nodeMap.GetNode("TriggerSoftware"))
            if not PySpin.IsAvailable(triggerCommand) or not PySpin.IsWritable(triggerCommand):
                print("Unable to trigger camera!")
                raise Exception()

            triggerCommand.Execute()

        except PySpin.SpinnakerException as se:
            print("Spinnaker exception during camera trigger %s" % se)
            return False

    # Returns camera to non-triggered streaming
    def resetTriggeredCamera(self):
        triggerMode = PySpin.CEnumerationPtr(self.nodeMap.GetNode('TriggerMode'))
        if not PySpin.IsAvailable(triggerMode) or not PySpin.IsReadable(triggerMode):
            print("Node Unavailable. Unable to disable tirgger mode. Exception")
            raise Exception()

        triggerModeOff = triggerMode.GetEntryByName('Off')

        if not PySpin.IsAvailable(triggerModeOff) or not PySpin.IsReadable(triggerModeOff):
            print("Enum Unavailable. Unable to disable trigger mode. Exception")
            raise Exception()

        triggerMode.SetIntValue(triggerModeOff.GetValue())
        print("Spin camera trigger unset")

    # Releases camera
    def tearDown(self):
        del self.camera
        self.cameraList.Clear()
        self.system.ReleaseInstance()


