import cv2
import depthai as dai
import numpy as np
import numpy.typing as npt

from src.fps_handler import FPSHandler
from src.utils import cv2_put_xyz, draw_box, extract_bounding_box

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
        "spatial": {
            "roi": {
                "xy_min": dai.Point2f(0.4, 0.4),
                "xy_max": dai.Point2f(0.6, 0.6)
            },
            "depth_thresholds": {
                "lower": 100,
                "upper": 10000
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
    spatial_cfg_orb = dai.SpatialLocationCalculatorConfigData()
    spatial_cfg_orb.roi = dai.Rect(
        CONFIG["pipeline"]["spatial"]["roi"]["xy_min"], CONFIG["pipeline"]["spatial"]["roi"]["xy_max"])
    spatial_cfg_orb.depthThresholds.lowerThreshold = CONFIG["pipeline"][
        "spatial"]["depth_thresholds"]["lower"]
    spatial_cfg_orb.depthThresholds.upperThreshold = CONFIG["pipeline"][
        "spatial"]["depth_thresholds"]["upper"]
    spatial_cfg_orb.calculationAlgorithm = CONFIG["pipeline"]["spatial"]["algo"]

    spatial_cfg_car = dai.SpatialLocationCalculatorConfigData()
    spatial_cfg_car.roi = dai.Rect(
        CONFIG["pipeline"]["spatial"]["roi"]["xy_min"], CONFIG["pipeline"]["spatial"]["roi"]["xy_max"])
    spatial_cfg_car.depthThresholds.lowerThreshold = CONFIG["pipeline"][
        "spatial"]["depth_thresholds"]["lower"]
    spatial_cfg_car.depthThresholds.upperThreshold = CONFIG["pipeline"][
        "spatial"]["depth_thresholds"]["upper"]
    spatial_cfg_car.calculationAlgorithm = CONFIG["pipeline"]["spatial"]["algo"]

    spatial_calc.inputConfig.setWaitForMessage(False)
    spatial_calc.initialConfig.addROI(spatial_cfg_orb)
    spatial_calc.initialConfig.addROI(spatial_cfg_car)

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
            maxSize=1,
            blocking=False
        )
        q_spatial_calc_config: dai.DataInputQueue = device.getInputQueue(  # type: ignore
            name="spatial_calc_config"
        )

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

        car_x1, car_y1, car_x2, car_y2 = 0, 0, 0, 0
        bounding_box_car = (car_x1, car_y1, car_x2, car_y2)

        orb_x1, orb_y1, orb_x2, orb_y2 = 0, 0, 0, 0
        bounding_box_orb = (orb_x1, orb_y1, orb_x2, orb_y2)

        #########################################
        #### Main host-side application loop ####
        #########################################
        while True:
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
                bounding_box_orb, bounding_box_car = bb

                orb_x1, orb_y1, orb_x2, orb_y2 = bounding_box_orb
                orb_xy_min = dai.Point2f(
                    orb_x1 / CONFIG["proc"]["image_size"][1],
                    orb_y1 / CONFIG["proc"]["image_size"][0]
                )
                orb_xy_max = dai.Point2f(
                    orb_x2 / CONFIG["proc"]["image_size"][1],
                    orb_y2 / CONFIG["proc"]["image_size"][0]
                )

                car_x1, car_y1, car_x2, car_y2 = bounding_box_car
                car_xy_min = dai.Point2f(
                    car_x1 / CONFIG["proc"]["image_size"][1],
                    car_y1 / CONFIG["proc"]["image_size"][0]
                )
                car_xy_max = dai.Point2f(
                    car_x2 / CONFIG["proc"]["image_size"][1],
                    car_y2 / CONFIG["proc"]["image_size"][0]
                )

                spatial_cfg_orb.roi = dai.Rect(orb_xy_min, orb_xy_max)
                spatial_cfg_car.roi = dai.Rect(car_xy_min, car_xy_max)
                cfg = dai.SpatialLocationCalculatorConfig()
                cfg.addROI(spatial_cfg_orb)
                cfg.addROI(spatial_cfg_car)
                q_spatial_calc_config.send(cfg)

            draw_box(frames["out"], bounding_box_orb, CONFIG, (0, 255, 255))
            cv2.putText(
                img=frames["out"],
                text="ORB",
                org=(orb_x1, orb_y1 - 10),
                fontFace=CONFIG["display"]["font"],
                fontScale=0.5,
                color=(0, 255, 255),
                lineType=CONFIG["display"]["line_type"]
            )

            draw_box(frames["out"], bounding_box_car, CONFIG, (0, 255, 255))
            cv2.putText(
                img=frames["out"],
                text="CAR",
                org=(car_x1, car_y1 - 10),
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
                cv2_put_xyz(
                    frame=frames["out"],
                    xyz=(
                        int(depth_data.spatialCoordinates.x),
                        int(depth_data.spatialCoordinates.y),
                        int(depth_data.spatialCoordinates.z)
                    ),
                    anchor=(xmax, ymin),
                    CONFIG=CONFIG
                )

            fps_handler.update()
            fps_handler.draw(frames["out"], CONFIG)

            cv2.imshow("Custom Object Tracker", frames["out"])

            if cv2.waitKey(1) == ord('q'):
                break
