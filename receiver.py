import depthai as dai

devices = dai.DeviceBootloader.getAllAvailableDevices()
print(devices)
