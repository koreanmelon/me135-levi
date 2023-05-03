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
CONFIG = {
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
        },
        "spatial_calc": {
            "roi": {
                "top_left": dai.Point2f(0.4, 0.4),
                "bottom_right": dai.Point2f(0.6, 0.6)
            },
            "depth_thresholds": {
                "lower_threshold": 100,
                "upper_threshold": 10000
            },
            "algo": dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        }
    },
    "proc": {
        "image_size": (720, 1280),
        "blur_size": (15, 15),
        "H_thresh": 10,
        "S_thresh": (250, 255),
        "V_thresh": (0, 255),
        "thresh": 25
    }
}


def extract_bounding_box(color_frame: npt.NDArray, bb_config: dict):
    frame = color_frame.copy()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    red_lower = np.array(
        [0, bb_config["S"][0], bb_config["V"][0]])
    red_upper = np.array(
        [bb_config["H"], bb_config["S"][1], bb_config["V"][1]])
    mask_right = cv2.inRange(hsv_frame, red_lower, red_upper)

    red_lower = np.array(
        [180 - bb_config["H"], bb_config["S"][0], bb_config["V"][0]])
    red_upper = np.array(
        [180, bb_config["S"][1], bb_config["V"][1]])
    mask_left = cv2.inRange(hsv_frame, red_lower, red_upper)

    mask = mask_right + mask_left  # type: ignore
    frame[np.where(mask == 0)] = 0

    # Post processing
    frame = cv2.blur(
        src=frame,
        ksize=bb_config["blur_size"]
    )
    ret, frame = cv2.threshold(
        src=frame,
        thresh=bb_config["thresh"],
        maxval=255,
        type=cv2.THRESH_BINARY
    )
    frame = cv2.cvtColor(
        src=frame,
        code=cv2.COLOR_RGB2GRAY
    )

    # Find contours
    contours, hierarchy = cv2.findContours(
        image=frame,
        mode=cv2.RETR_TREE,
        method=cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) > 0:
        cnt = contours[0]
        x, y, w, h = cv2.boundingRect(cnt)
        x2, y2 = x + w, y + h
        return x, y, x2, y2

    return None


def draw_box(frame: npt.NDArray, bbox: tuple, color=None):
    x, y, x2, y2 = bbox
    return cv2.rectangle(
        img=frame,
        pt1=(x, y),
        pt2=(x2, y2),
        color=CONFIG["display"]["line_color"] if not color else color,
        thickness=CONFIG["display"]["line_thickness"]
    )


if __name__ == "__main__":
    ##################
    #### Pipeline ####
    ##################
    pipeline = dai.Pipeline()

    # Create nodes
    camera_color = pipeline.createColorCamera()
    xout_color = pipeline.createXLinkOut()
    mono_left = pipeline.createMonoCamera()
    mono_right = pipeline.createMonoCamera()
    stereo = pipeline.createStereoDepth()
    spatial_calc = pipeline.createSpatialLocationCalculator()
    xout_depth = pipeline.createXLinkOut()
    xout_spatial_data = pipeline.createXLinkOut()
    xin_spatial_calc_config = pipeline.createXLinkIn()

    # Color camera config
    camera_color.setBoardSocket(dai.CameraBoardSocket.RGB)
    camera_color.setResolution(CONFIG["pipeline"]["color_cam"]["resolution"])
    camera_color.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camera_color.setIspScale(CONFIG["pipeline"]["color_cam"]["isp_scale"])
    camera_color.setInterleaved(False)

    xout_color.setStreamName("color")

    # Mono camera config
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_left.setResolution(CONFIG["pipeline"]["mono_cam"]["resolution"])
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_right.setResolution(CONFIG["pipeline"]["mono_cam"]["resolution"])

    # Stereo depth
    stereo.setDefaultProfilePreset(
        dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.initialConfig.setConfidenceThreshold(255)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)

    # Spatial location calculator
    spatial_config = dai.SpatialLocationCalculatorConfigData()
    spatial_config.roi = dai.Rect(
        CONFIG["pipeline"]["spatial_calc"]["roi"]["top_left"],
        CONFIG["pipeline"]["spatial_calc"]["roi"]["bottom_right"])
    spatial_config.depthThresholds.lowerThreshold = CONFIG["pipeline"][
        "spatial_calc"]["depth_thresholds"]["lower_threshold"]
    spatial_config.depthThresholds.upperThreshold = CONFIG["pipeline"][
        "spatial_calc"]["depth_thresholds"]["upper_threshold"]
    spatial_config.calculationAlgorithm = CONFIG["pipeline"]["spatial_calc"]["algo"]

    spatial_calc.inputConfig.setWaitForMessage(False)
    spatial_calc.initialConfig.addROI(spatial_config)

    xout_depth.setStreamName("depth")
    xout_spatial_data.setStreamName("spatial_data")
    xin_spatial_calc_config.setStreamName("spatial_calc_config")

    # Linking
    camera_color.isp.link(xout_color.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(spatial_calc.inputDepth)

    spatial_calc.passthroughDepth.link(xout_depth.input)
    spatial_calc.out.link(xout_spatial_data.input)

    xin_spatial_calc_config.out.link(spatial_calc.inputConfig)

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
        q_color: dai.DataOutputQueue = device.getOutputQueue(  # type: ignore
            name="color",
            maxSize=1,
            blocking=False
        )
        q_depth: dai.DataOutputQueue = device.getOutputQueue(  # type: ignore
            name="depth",
            maxSize=1,
            blocking=False
        )
        q_spatial_calc: dai.DataOutputQueue = device.getOutputQueue(  # type: ignore
            name="spatial_data",
            maxSize=4,
            blocking=False
        )
        q_spatial_calc_config: dai.DataInputQueue = device.getInputQueue(  # type: ignore
            name="spatial_calc_config"
        )

        xy_min = CONFIG["pipeline"]["spatial_calc"]["roi"]["top_left"]
        xy_max = CONFIG["pipeline"]["spatial_calc"]["roi"]["bottom_right"]

        fps_handler = FPSHandler()

        empty_frame: npt.NDArray = np.zeros_like(
            (CONFIG["proc"]["image_size"][0],
             CONFIG["proc"]["image_size"][1], 3),
            dtype=np.uint8
        )
        frame_types = {
            "color": empty_frame,
            "out": empty_frame,
            "depth": empty_frame,
            "depth_color": empty_frame
        }
        annotations = []

        x, y, x2, y2 = 0, 0, 0, 0
        # Main host-side application loop
        while True:
            fps_handler.update()

            in_color = q_color.get()
            frame_types["color"] = in_color.getCvFrame()  # type: ignore
            frame_types["out"] = frame_types["color"].copy()

            bounding_box = extract_bounding_box(
                frame_types["color"],
                bb_config={
                    "H": CONFIG["proc"]["H_thresh"],
                    "S": CONFIG["proc"]["S_thresh"],
                    "V": CONFIG["proc"]["V_thresh"],
                    "thresh": CONFIG["proc"]["thresh"],
                    "blur_size": CONFIG["proc"]["blur_size"],
                }
            )

            if bounding_box is not None:
                x, y, x2, y2 = bounding_box
                xy_min = dai.Point2f(
                    x / CONFIG["proc"]["image_size"][1],
                    y / CONFIG["proc"]["image_size"][0]
                )
                xy_max = dai.Point2f(
                    x2 / CONFIG["proc"]["image_size"][1],
                    y2 / CONFIG["proc"]["image_size"][0]
                )

                spatial_config.roi = dai.Rect(xy_min, xy_max)
                cfg = dai.SpatialLocationCalculatorConfig()
                cfg.addROI(spatial_config)
                q_spatial_calc_config.send(cfg)

            in_depth = q_depth.get()
            frame_types["depth"] = in_depth.getFrame()  # type: ignore

            in_spatial_data = q_spatial_calc.get()
            spatial_data = in_spatial_data.getSpatialLocations()  # type: ignore

            depth_downscaled = frame_types["depth"][::4]
            min_depth = np.percentile(
                depth_downscaled[depth_downscaled != 0],
                1
            )
            max_depth = np.percentile(
                depth_downscaled,
                99
            )
            frame_types["depth_color"] = np.interp(
                x=frame_types["depth"],
                xp=(min_depth, max_depth),
                fp=(0, 255)
            ).astype(np.uint8)
            frame_types["depth_color"] = cv2.applyColorMap(
                frame_types["depth_color"], cv2.COLORMAP_HOT)

            for depth_data in spatial_data:
                roi = depth_data.config.roi
                roi = roi.denormalize(
                    width=frame_types["depth_color"].shape[1],
                    height=frame_types["depth_color"].shape[0]
                )
                xmin = int(roi.topLeft().x)
                ymin = int(roi.topLeft().y)
                xmax = int(roi.bottomRight().x)
                ymax = int(roi.bottomRight().y)

                depth_min = depth_data.depthMin
                depth_max = depth_data.depthMax

                draw_box(frame_types["out"], (xmin, ymin, xmax, ymax))
                cv2.putText(
                    img=frame_types["out"],
                    text=f"X: {int(depth_data.spatialCoordinates.x)} mm",
                    org=(xmax + 10, ymin + 20),
                    fontFace=CONFIG["display"]["font"],
                    fontScale=0.5,
                    color=CONFIG["display"]["line_color"],
                    lineType=CONFIG["display"]["line_type"]
                )
                cv2.putText(
                    img=frame_types["out"],
                    text=f"Y: {int(depth_data.spatialCoordinates.y)} mm",
                    org=(xmax + 10, ymin + 35),
                    fontFace=CONFIG["display"]["font"],
                    fontScale=0.5,
                    color=CONFIG["display"]["line_color"],
                    lineType=CONFIG["display"]["line_type"]
                )
                cv2.putText(
                    img=frame_types["out"],
                    text=f"Z: {int(depth_data.spatialCoordinates.z)} mm",
                    org=(xmax + 10, ymin + 50),
                    fontFace=CONFIG["display"]["font"],
                    fontScale=0.5,
                    color=CONFIG["display"]["line_color"],
                    lineType=CONFIG["display"]["line_type"]
                )

                draw_box(frame_types["out"], (x, y, x2, y2), (0, 255, 255))
                cv2.putText(
                    img=frame_types["out"],
                    text=f"X: {x}, Y: {y}",
                    org=(x, y - 10),
                    fontFace=CONFIG["display"]["font"],
                    fontScale=0.5,
                    color=(0, 255, 255),
                    lineType=CONFIG["display"]["line_type"]
                )

            cv2.putText(
                img=frame_types["out"],
                text=f"FPS: {int(fps_handler.average())}",
                org=(5, 30),
                fontFace=CONFIG["display"]["font"],
                fontScale=0.5,
                color=CONFIG["display"]["line_color"],
                lineType=CONFIG["display"]["line_type"]
            )

            # cv2.imshow("Custom Object Tracker", out_frame)
            cv2.imshow("Custom Object Tracker", np.concatenate(
                (frame_types["depth_color"], frame_types["out"]),
                axis=0)
            )

            # if frame_idx >= wait_frames:
            # annotations.append(
            #     [f"{frame_idx - wait_frames:04}.jpg", image_size[1], image_size[0], "orb", x, y, x2, y2])
            # cv2.imwrite(
            #     f"./datasets/dataset_05/{frame_idx - wait_frames:04}.jpg",
            #     raw_frame
            # )

            if cv2.waitKey(1) == ord('q'):
                break

        # df = pd.DataFrame(
        #     annotations,
        #     columns=["filename", "width", "height", "class", "xmin", "ymin", "xmax", "ymax"])
        # df.to_csv("./datasets/dataset_05.csv", index=False)
