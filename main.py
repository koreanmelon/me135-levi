from datetime import timedelta

import blobconverter
import cv2
import depthai as dai
import numpy as np
import numpy.typing as npt
import pandas as pd

from src.fps_handler import FPSHandler

################
#### Config ####
################
config = {
    "display": {
        "font": cv2.FONT_HERSHEY_SIMPLEX,
        "line_type": cv2.LINE_AA,
        "line_color": (0, 255, 0),
        "line_thickness": 2
    },
    "fps": {
        "max_len": 10
    },
    "pipeline": {
        "color_cam": {
            "resolution": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            "fps": 30,
            "isp_scale": (2, 3),
            "orientation": dai.CameraImageOrientation.VERTICAL_FLIP,
            "board_socket": dai.CameraBoardSocket.RGB
        },
        "mono_cam": {
            "resolution": dai.MonoCameraProperties.SensorResolution.THE_480_P,
            "fps": 30
        }
    }
}

image_size = (720, 1280)  # (height, width)
X_shape = (3, *image_size)
blur_size = (25, 25)

H_thresh = 10
S_thresh = (250, 255)
V_thresh = (0, 255)
thresh = 25

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


def crop_to_square(frame):
    height = frame.shape[0]
    width = frame.shape[1]
    delta = int((width-height) / 2)
    return frame[0:height, delta:width-delta]


class HostSync:
    def __init__(self):
        self.arrays = {}

    def add_msg(self, name, msg):
        if name not in self.arrays:
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


def setup_pipeline(config):
    # Create pipeline
    pipeline = dai.Pipeline()

    # RGB camera
    camera_color = pipeline.createColorCamera()
    camera_color.setInterleaved(False)
    camera_color.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camera_color.setResolution(config["pipeline"]["color_cam"]["resolution"])
    camera_color.setIspScale(config["pipeline"]["color_cam"]["isp_scale"])
    camera_color.setPreviewSize(*INPUT_SHAPE)
    camera_color.setBoardSocket(dai.CameraBoardSocket.RGB)

    xout_color = pipeline.create(dai.node.XLinkOut)
    xout_color.setStreamName("color")

    # Depth neural network
    depth_nn = pipeline.create(dai.node.NeuralNetwork)
    depth_nn.setBlob(blob)
    depth_nn.input.setBlocking(False)
    depth_nn.setNumInferenceThreads(2)

    xout_nn = pipeline.create(dai.node.XLinkOut)
    xout_nn.setStreamName("nn")

    # Mono cameras
    mono_left = pipeline.createMonoCamera()
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_left.setResolution(config["pipeline"]["mono_cam"]["resolution"])

    mono_right = pipeline.createMonoCamera()
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_right.setResolution(config["pipeline"]["mono_cam"]["resolution"])

    # Stereo depth
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(
        dai.node.StereoDepth.PresetMode.HIGH_DENSITY
    )
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

    xout_disp = pipeline.create(dai.node.XLinkOut)
    xout_disp.setStreamName("disparity")

    # Spatial location calculator
    spatial_loc_calc = pipeline.createSpatialLocationCalculator()
    top_left = dai.Point2f(0.4, 0.4)
    bottom_right = dai.Point2f(0.6, 0.6)

    spatial_config = dai.SpatialLocationCalculatorConfigData()
    spatial_config.depthThresholds.lowerThreshold = 100
    spatial_config.depthThresholds.upperThreshold = 10000
    calc_algo = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
    spatial_config.roi = dai.Rect(top_left, bottom_right)

    spatial_loc_calc.inputConfig.setWaitForMessage(False)
    spatial_loc_calc.initialConfig.addROI(spatial_config)

    xout_depth = pipeline.createXLinkOut()
    xout_depth.setStreamName("depth")

    xout_spatial_data = pipeline.createXLinkOut()
    xout_spatial_data.setStreamName("spatialData")

    xin_spatial_calc_config = pipeline.createXLinkIn()
    xin_spatial_calc_config.setStreamName("spatialCalcConfig")

    # Linking
    camera_color.preview.link(depth_nn.input)
    camera_color.isp.link(xout_color.input)

    depth_nn.out.link(xout_nn.input)

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    stereo.disparity.link(xout_disp.input)

    spatial_loc_calc.passthroughDepth.link(xout_depth.input)
    stereo.depth.link(spatial_loc_calc.inputDepth)

    spatial_loc_calc.out.link(xout_spatial_data.input)
    xin_spatial_calc_config.out.link(spatial_loc_calc.inputConfig)

    # Return nodes
    nodes: dict = {
        "camera_color": camera_color,
        "xout_color": xout_color,
        "depth_nn": depth_nn,
        "xout_nn": xout_nn,
        "mono_left": mono_left,
        "mono_right": mono_right,
        "stereo": stereo,
        "xout_disp": xout_disp,
        "spatial_loc_calc": spatial_loc_calc,
        "spatial_config": spatial_config,
        "xout_depth": xout_depth,
        "xout_spatial_data": xout_spatial_data,
        "xin_spatial_calc_config": xin_spatial_calc_config
    }

    return pipeline, nodes


if __name__ == "__main__":
    pipeline, nodes = setup_pipeline(config)
    stereo: dai.node.StereoDepth = nodes["stereo"]  # type: ignore
    spatial_config: dai.SpatialLocationCalculatorConfigData = nodes["spatial_config"]

    ###########################
    #### Device Connection ####
    ###########################
    with dai.Device() as device:
        cams = device.getConnectedCameras()
        depth_enabled = dai.CameraBoardSocket.LEFT in cams and dai.CameraBoardSocket.RIGHT in cams
        if not depth_enabled:
            raise RuntimeError(
                f"Device lacks depth capabilities. Available cameras: {cams}")

        # !!!Remove this if the device is already running a pipeline!!!
        device.startPipeline(pipeline)

        # Prioritize the latest input data
        queue_color: dai.DataOutputQueue = device.getOutputQueue(  # type: ignore
            name="color",
            maxSize=1,
            blocking=False
        )

        queue_disparity: dai.DataOutputQueue = device.getOutputQueue(  # type: ignore
            name="disparity",
            maxSize=1,
            blocking=False
        )

        queue_nn: dai.DataOutputQueue = device.getOutputQueue(  # type: ignore
            name="nn",
            maxSize=1,
            blocking=False
        )

        topLeft = dai.Point2f(0.4, 0.4)
        bottomRight = dai.Point2f(0.6, 0.6)

        depthQueue = device.getOutputQueue(  # type: ignore
            name="depth", maxSize=4, blocking=False)
        spatialCalcQueue = device.getOutputQueue(  # type: ignore
            name="spatialData", maxSize=4, blocking=False)
        spatialCalcConfigInQueue = device.getInputQueue(  # type: ignore
            "spatialCalcConfig")

        color = (255, 255, 255)

        fps_handler = FPSHandler()
        sync = HostSync()

        raw_frame: npt.NDArray = np.zeros_like(
            (image_size[0], image_size[1], 3),
            dtype=np.uint8
        )

        frames = {}

        frame_idx = 0
        wait_frames = 30
        num_frames = 300

        annotations = []

        # Main host-side application loop
        # for frame_idx in range(num_frames + wait_frames):
        while True:
            newConfig = False
            msgs = False
            if queue_color.has():
                msgs = msgs or sync.add_msg("color", queue_color.get())
            if queue_disparity.has():
                msgs = msgs or sync.add_msg("depth", queue_disparity.get())
            if queue_nn.has():
                msgs = msgs or sync.add_msg("nn", queue_nn.get())

            if msgs:
                fps_handler.update()

                frame = msgs["color"].getCvFrame()
                
                inDepth = depthQueue.get()  # Blocking call, will wait until a new data has arrived

                depthFrame = inDepth.getFrame()  # depthFrame values are in millimeters

                depth_downscaled = depthFrame[::4]
                min_depth = np.percentile(
                    depth_downscaled[depth_downscaled != 0], 1)
                max_depth = np.percentile(depth_downscaled, 99)
                depthFrameColor = np.interp(
                    depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
                depthFrameColor = cv2.applyColorMap(
                    depthFrameColor, cv2.COLORMAP_HOT)

                spatialData = spatialCalcQueue.get().getSpatialLocations()
                for depthData in spatialData:
                    roi = depthData.config.roi
                    roi = roi.denormalize(
                        width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
                    xmin = int(roi.topLeft().x)
                    ymin = int(roi.topLeft().y)
                    xmax = int(roi.bottomRight().x)
                    ymax = int(roi.bottomRight().y)

                    depthMin = depthData.depthMin
                    depthMax = depthData.depthMax

                    cv2.rectangle(frame, (xmin, ymin),
                                (xmax, ymax), color, 1)
                    cv2.putText(frame, f"X: {int(depthData.spatialCoordinates.x)} mm", (
                        xmin + 10, ymin + 20), config["display"]["font"], 0.5, color)
                    cv2.putText(frame, f"Y: {int(depthData.spatialCoordinates.y)} mm", (
                        xmin + 10, ymin + 35), config["display"]["font"], 0.5, color)
                    cv2.putText(frame, f"Z: {int(depthData.spatialCoordinates.z)} mm", (
                        xmin + 10, ymin + 50), config["display"]["font"], 0.5, color)

                    frames["depth_frame_color"] = depthFrameColor

                frames["raw_frame"] = frame
                frames["out_frame"] = frame.copy()

                hsv_frame = cv2.cvtColor(
                    frames["raw_frame"], cv2.COLOR_BGR2HSV)
                frames["hsv_frame"] = hsv_frame

                # right mask H
                red_lower = np.array([0, S_thresh[0], V_thresh[0]])
                red_upper = np.array([H_thresh, S_thresh[1], V_thresh[1]])
                mask_right = cv2.inRange(hsv_frame, red_lower, red_upper)

                # left mask H
                red_lower = np.array(
                    [180 - H_thresh, S_thresh[0], V_thresh[0]])
                red_upper = np.array([180, S_thresh[1], V_thresh[1]])
                mask_left = cv2.inRange(hsv_frame, red_lower, red_upper)

                mask = mask_right + mask_left  # type: ignore
                frames["mask"] = mask

                # Set output frame to zero everywhere except the mask
                output_rgb = frames["raw_frame"].copy()
                output_rgb[np.where(mask == 0)] = 0

                # Post processing
                proc_frame = output_rgb
                proc_frame = cv2.blur(
                    src=proc_frame,
                    ksize=blur_size
                )
                ret, proc_frame = cv2.threshold(
                    src=proc_frame,
                    thresh=thresh,
                    maxval=255,
                    type=cv2.THRESH_BINARY
                )
                proc_frame = cv2.cvtColor(
                    src=proc_frame,
                    code=cv2.COLOR_RGB2GRAY
                )
                frames["proc_frame"] = proc_frame

                # Find contours
                contours, hierarchy = cv2.findContours(
                    image=proc_frame,
                    mode=cv2.RETR_TREE,
                    method=cv2.CHAIN_APPROX_SIMPLE
                )

                x, y, x2, y2 = 0, 0, 0, 0
                if len(contours) > 0:
                    cnt = contours[0]
                    x, y, w, h = cv2.boundingRect(cnt)
                    x2, y2 = x + w, y + h
                    newConfig = True
                    topLeft = dai.Point2f(x, y)
                    bottomRight = dai.Point2f(x2, y2)

                    # frames["out_frame"] = cv2.rectangle(
                    #     img=frames["out_frame"],
                    #     pt1=(x, y),
                    #     pt2=(x+w, y+h),
                    #     color=config["display"]["line_color"],
                    #     thickness=2,
                    #     lineType=config["display"]["line_type"]
                    # )
                    # frames["out_frame"] = cv2.putText(
                    #     img=frames["out_frame"],
                    #     text=f"X: {x}, Y: {y}",
                    #     org=(x, y-10),
                    #     fontFace=config["display"]["font"],
                    #     fontScale=1,
                    #     color=config["display"]["line_color"],
                    #     thickness=1,
                    #     lineType=config["display"]["line_type"]
                    # )

                cv2.putText(
                    img=frames["out_frame"],
                    text=f"FPS: {int(fps_handler.average())}",
                    org=(5, 30),
                    fontFace=config["display"]["font"],
                    fontScale=1,
                    color=config["display"]["line_color"],
                    thickness=1,
                    lineType=config["display"]["line_type"]
                )

                frames["proc_frame"] = np.tile(
                    np.expand_dims(frames["proc_frame"], axis=2),
                    (1, 1, 3)
                )
                ff1 = np.concatenate(
                    (frames["out_frame"], frames["raw_frame"]),
                    axis=0
                )
                ff2 = np.concatenate(
                    (frames["proc_frame"], frames["hsv_frame"]),
                    axis=0
                )
                final_frame = np.concatenate(
                    (ff1, ff2),
                    axis=1
                )
                # cv2.imshow("Custom Object Tracker", final_frame)
                cv2.imshow("Custom Object Tracker", frames["out_frame"])
                # cv2.imshow("Custom Object Tracker",
                #            frames["depth_frame_color"])

                if frame_idx >= wait_frames:
                    annotations.append(
                        [f"{frame_idx - wait_frames:04}.jpg", image_size[1], image_size[0], "orb", x, y, x2, y2])
                    # cv2.imwrite(
                    #     f"./datasets/dataset_05/{frame_idx - wait_frames:04}.jpg",
                    #     raw_frame
                    # )

            if newConfig:
                spatial_config.roi = dai.Rect(topLeft, bottomRight)
                spatial_config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEAN
                cfg = dai.SpatialLocationCalculatorConfig()
                cfg.addROI(spatial_config)
                spatialCalcConfigInQueue.send(cfg)
                newConfig = False

            if cv2.waitKey(1) == ord('q'):
                break

        # df = pd.DataFrame(
        #     annotations,
        #     columns=["filename", "width", "height", "class", "xmin", "ymin", "xmax", "ymax"])
        # df.to_csv("./datasets/dataset_05.csv", index=False)
