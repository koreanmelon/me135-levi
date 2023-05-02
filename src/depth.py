import time
from datetime import timedelta

import blobconverter
import cv2
import depthai as dai
import numpy as np

jet_custom = cv2.applyColorMap(
    np.arange(256, dtype=np.uint8),
    cv2.COLORMAP_JET
)
jet_custom[0] = [0, 0, 0]

blob = dai.OpenVINO.Blob(
    blobconverter.from_zoo(
        name="deeplab_v3_mnv2_256x256",
        zoo_type="depthai",
        shaves=6
    )
)
INPUT_SHAPE = blob.networkInputs['Input'].dims[:2]
TARGET_SHAPE = (400, 400)


def decode_deeplabv3p(output_tensor):
    class_colors = [[0, 0, 0],  [0, 255, 0]]
    class_colors = np.asarray(class_colors, dtype=np.uint8)

    output = output_tensor.reshape(*INPUT_SHAPE)
    output_colors = np.take(class_colors, output, axis=0)
    return output_colors


def get_multiplier(output_tensor):
    class_binary = [[0], [1]]
    class_binary = np.asarray(class_binary, dtype=np.uint8)
    output = output_tensor.reshape(*INPUT_SHAPE)
    output_colors = np.take(class_binary, output, axis=0)
    return output_colors


class FPSHandler:
    def __init__(self):
        self.timestamp = time.time()
        self.start = time.time()
        self.frame_cnt = 0

    def next_iter(self):
        self.timestamp = time.time()
        self.frame_cnt += 1

    def fps(self):
        return self.frame_cnt / (self.timestamp - self.start)


class HostSync:
    def __init__(self):
        self.arrays = {}

    def add_msg(self, name, msg):
        if not name in self.arrays:
            self.arrays[name] = []
        self.arrays[name].append({'msg': msg})
        ts = msg.getTimestamp()
        synced = {}
        for name, arr in self.arrays.items():
            for i, obj in enumerate(arr):
                time_diff = abs(obj['msg'].getTimestamp() - ts)
                if time_diff < timedelta(milliseconds=33):
                    synced[name] = obj['msg']
                    # print(f"{name}: {i}/{len(arr)}")
                    break
        if len(synced) == 3:
            def remove(t1, t2):
                return timedelta(milliseconds=500) < abs(t1 - t2)
            # Remove old msgs
            for name, arr in self.arrays.items():
                for i, obj in enumerate(arr):
                    if remove(obj['msg'].getTimestamp(), ts):
                        arr.remove(obj)
                    else:
                        break
            return synced
        return False


def crop_to_square(frame):
    height = frame.shape[0]
    width = frame.shape[1]
    delta = int((width-height) / 2)
    return frame[0:height, delta:width-delta]


pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.ColorCamera)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setIspScale(2, 3)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)

cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.setPreviewSize(*INPUT_SHAPE)
cam.setInterleaved(False)

isp_xout = pipeline.create(dai.node.XLinkOut)
isp_xout.setStreamName("cam")
cam.isp.link(isp_xout.input)

detection_nn = pipeline.create(dai.node.NeuralNetwork)
detection_nn.setBlob(blob)
detection_nn.input.setBlocking(False)
detection_nn.setNumInferenceThreads(2)
cam.preview.link(detection_nn.input)

xout_nn = pipeline.create(dai.node.XLinkOut)
xout_nn.setStreamName("nn")
detection_nn.out.link(xout_nn.input)

left = pipeline.create(dai.node.MonoCamera)
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
left.setBoardSocket(dai.CameraBoardSocket.LEFT)
right = pipeline.create(dai.node.MonoCamera)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
left.out.link(stereo.left)
right.out.link(stereo.right)

xout_disp = pipeline.create(dai.node.XLinkOut)
xout_disp.setStreamName("disparity")
stereo.disparity.link(xout_disp.input)

with dai.Device(pipeline, usb2Mode=True) as device:
    cams = device.getConnectedCameras()
    depth_enabled = dai.CameraBoardSocket.LEFT in cams and dai.CameraBoardSocket.RIGHT in cams
    if not depth_enabled:
        raise RuntimeError(
            "Unable to run this experiment on device without depth capabilities! (Available cameras: {})".format(
                cams)
        )

    q_color = device.getOutputQueue(  # type: ignore
        name="cam", maxSize=4, blocking=False
    )
    q_disp = device.getOutputQueue(  # type: ignore
        name="disparity", maxSize=4, blocking=False
    )
    q_nn = device.getOutputQueue(  # type: ignore
        name="nn", maxSize=4, blocking=False
    )

    fps = FPSHandler()
    sync = HostSync()
    disp_frame = None
    disp_multiplier = 255 / stereo.initialConfig.getMaxDisparity()

    frame = None
    depth = None
    depth_weighted = None
    frames = {}

    while True:
        msgs = False
        if q_color.has():
            msgs = msgs or sync.add_msg("color", q_color.get())
        if q_disp.has():
            msgs = msgs or sync.add_msg("depth", q_disp.get())
        if q_nn.has():
            msgs = msgs or sync.add_msg("nn", q_nn.get())

        if msgs:
            fps.next_iter()
            layer1 = msgs['nn'].getFirstLayerInt32()
            lay1 = np.asarray(layer1, dtype=np.int32).reshape(*INPUT_SHAPE)
            output_colors = decode_deeplabv3p(lay1)

            output_colors = cv2.resize(output_colors, TARGET_SHAPE)

            frame = msgs["color"].getCvFrame()
            frame = crop_to_square(frame)
            frame = cv2.resize(frame, TARGET_SHAPE)
            frames['frame'] = frame
            frame = cv2.addWeighted(frame, 1, output_colors, 0.5, 0)
            cv2.putText(
                frame,
                f"FPS: {fps.fps():.2f}",
                (2, frame.shape[0] - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color=(255, 255, 255)
            )
            frames['colored_frame'] = frame

            disp_frame = msgs["depth"].getFrame()
            disp_frame = (disp_frame * disp_multiplier).astype(np.uint8)
            disp_frame = crop_to_square(disp_frame)
            disp_frame = cv2.resize(disp_frame, TARGET_SHAPE)

            frames['depth'] = cv2.applyColorMap(disp_frame, jet_custom)

            multiplier = get_multiplier(lay1)
            multiplier = cv2.resize(multiplier, TARGET_SHAPE)
            depth_overlay = disp_frame * multiplier  # type: ignore
            frames['cutout'] = cv2.applyColorMap(depth_overlay, jet_custom)

        if len(frames) == 4:
            show = np.concatenate(
                (frames['colored_frame'], frames['cutout'], frames['depth']),
                axis=1
            )
            cv2.imshow("Colored | Cutout | Depth", show)

        if cv2.waitKey(1) == ord('q'):
            break
