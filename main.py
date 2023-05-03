import cv2
import depthai as dai
import numpy as np
import numpy.typing as npt

from src.fps_handler import FPSHandler
from src.utils import draw_box, extract_bounding_box

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
            "fps": 35,
            "isp_scale": (2, 3),
            "orientation": dai.CameraImageOrientation.VERTICAL_FLIP,
            "board_socket": dai.CameraBoardSocket.RGB
        },
        "mono_cam": {
            "resolution": dai.MonoCameraProperties.SensorResolution.THE_480_P,
            "fps": 35
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
    camera_color.setFps(CONFIG["pipeline"]["color_cam"]["fps"])

    xout_color.setStreamName("color")

    # Mono camera config
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_left.setResolution(CONFIG["pipeline"]["mono_cam"]["resolution"])
    mono_left.setFps(CONFIG["pipeline"]["mono_cam"]["fps"])
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_right.setResolution(CONFIG["pipeline"]["mono_cam"]["resolution"])
    mono_right.setFps(CONFIG["pipeline"]["mono_cam"]["fps"])

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
        frames = {
            "color": empty_frame,
            "out": empty_frame,
            "depth": empty_frame,
            "depth_color": empty_frame
        }

        x, y, x2, y2 = 0, 0, 0, 0
        bounding_box = (x, y, x2, y2)

        #########################################
        #### Main host-side application loop ####
        #########################################
        while True:
            fps_handler.update()

            in_color = q_color.get()
            frames["color"] = in_color.getCvFrame()  # type: ignore
            frames["out"] = frames["color"].copy()

            bb = extract_bounding_box(
                frames["color"],
                bb_config={
                    "H": CONFIG["proc"]["H_thresh"],
                    "S": CONFIG["proc"]["S_thresh"],
                    "V": CONFIG["proc"]["V_thresh"],
                    "thresh": CONFIG["proc"]["thresh"],
                    "blur_size": CONFIG["proc"]["blur_size"],
                }
            )

            if bb is not None:
                bounding_box = bb
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

            draw_box(frames["out"], bounding_box, CONFIG, (0, 255, 255))
            cv2.putText(
                img=frames["out"],
                text=f"X: {x}, Y: {y}",
                org=(x, y - 10),
                fontFace=CONFIG["display"]["font"],
                fontScale=0.5,
                color=(0, 255, 255),
                lineType=CONFIG["display"]["line_type"]
            )

            in_depth = q_depth.get()
            frames["depth"] = in_depth.getFrame()  # type: ignore

            in_spatial_data = q_spatial_calc.get()
            spatial_data = in_spatial_data.getSpatialLocations()  # type: ignore

            depth_downscaled = frames["depth"][::4]
            min_depth = np.percentile(
                depth_downscaled[depth_downscaled != 0], 1)
            max_depth = np.percentile(depth_downscaled, 99)
            frames["depth_color"] = np.interp(
                x=frames["depth"],
                xp=(min_depth, max_depth),
                fp=(0, 255)
            ).astype(np.uint8)
            frames["depth_color"] = cv2.applyColorMap(
                frames["depth_color"], cv2.COLORMAP_HOT)

            for depth_data in spatial_data:
                roi = depth_data.config.roi
                roi = roi.denormalize(
                    width=frames["depth_color"].shape[1],
                    height=frames["depth_color"].shape[0]
                )
                xmin = int(roi.topLeft().x)
                ymin = int(roi.topLeft().y)
                xmax = int(roi.bottomRight().x)
                ymax = int(roi.bottomRight().y)

                depth_min = depth_data.depthMin
                depth_max = depth_data.depthMax

                draw_box(frames["out"], (xmin, ymin, xmax, ymax), CONFIG)
                cv2.putText(
                    img=frames["out"],
                    text=f"X: {int(depth_data.spatialCoordinates.x)} mm",
                    org=(xmax + 10, ymin + 20),
                    fontFace=CONFIG["display"]["font"],
                    fontScale=0.5,
                    color=CONFIG["display"]["line_color"],
                    lineType=CONFIG["display"]["line_type"]
                )
                cv2.putText(
                    img=frames["out"],
                    text=f"Y: {int(depth_data.spatialCoordinates.y)} mm",
                    org=(xmax + 10, ymin + 35),
                    fontFace=CONFIG["display"]["font"],
                    fontScale=0.5,
                    color=CONFIG["display"]["line_color"],
                    lineType=CONFIG["display"]["line_type"]
                )
                cv2.putText(
                    img=frames["out"],
                    text=f"Z: {int(depth_data.spatialCoordinates.z)} mm",
                    org=(xmax + 10, ymin + 50),
                    fontFace=CONFIG["display"]["font"],
                    fontScale=0.5,
                    color=CONFIG["display"]["line_color"],
                    lineType=CONFIG["display"]["line_type"]
                )

            fps_handler.draw(frames["out"], CONFIG)

            cv2.imshow("Custom Object Tracker", frames["out"])

            if cv2.waitKey(1) == ord('q'):
                break
