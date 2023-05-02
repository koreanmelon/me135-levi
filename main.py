import cv2
import depthai as dai
import numpy as np
import numpy.typing as npt
import pandas as pd

from fps_handler import FPSHandler

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
        "color_camera": {
            "resolution": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            "fps": 30,
            "isp_scale": (2, 3),
            "orientation": dai.CameraImageOrientation.VERTICAL_FLIP,
            "board_socket": dai.CameraBoardSocket.RGB
        },
        "mono_camera": {
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

if __name__ == "__main__":
    # Create pipeline
    pipeline = dai.Pipeline()

    # RGB camera
    camera_color = pipeline.create(dai.node.ColorCamera)
    camera_color.setInterleaved(False)
    camera_color.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camera_color.setResolution(
        config["pipeline"]["color_camera"]["resolution"]
    )
    camera_color.setIspScale(config["pipeline"]["color_camera"]["isp_scale"])
    camera_color.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)
    camera_color.setBoardSocket(dai.CameraBoardSocket.RGB)

    xout_color = pipeline.create(dai.node.XLinkOut)
    xout_color.setStreamName("color")

    camera_color.isp.link(xout_color.input)

    # Mono camera
    camera_left = pipeline.create(dai.node.MonoCamera)
    camera_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    camera_left.setResolution(
        dai.MonoCameraProperties.SensorResolution.THE_480_P)

    camera_right = pipeline.create(dai.node.MonoCamera)
    camera_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    camera_right.setResolution(
        dai.MonoCameraProperties.SensorResolution.THE_480_P)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(
        dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    camera_left.out.link(stereo.left)
    camera_right.out.link(stereo.right)

    xout_disp = pipeline.create(dai.node.XLinkOut)
    xout_disp.setStreamName("disparity")
    stereo.disparity.link(xout_disp.input)

    ###########################
    #### Device Connection ####
    ###########################
    with dai.Device(pipeline=pipeline) as device:
        cams = device.getConnectedCameras()

        depth_enabled = dai.CameraBoardSocket.LEFT in cams and dai.CameraBoardSocket.RIGHT in cams
        if not depth_enabled:
            raise RuntimeError(
                f"Unable to run this experiment on device without depth capabilities! (Available cameras: {cams})"
            )

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

        raw_frame: npt.NDArray = np.zeros_like(
            (image_size[0], image_size[1], 3),
            dtype=np.uint8
        )

        fps_handler = FPSHandler()

        frames = {}
        frame_idx = 0
        wait_frames = 30
        num_frames = 300

        annotations = []

        # Main host-side application loop
        for frame_idx in range(num_frames + wait_frames):
            # while True:
            in_rgb: dai.ImgFrame = queue_color.get()  # type: ignore
            raw_frame = in_rgb.getCvFrame()  # type: ignore
            frames["raw_frame"] = raw_frame
            frames["out_frame"] = raw_frame.copy()

            hsv_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2HSV)
            frames["hsv_frame"] = hsv_frame

            # right mask H
            red_lower = np.array([0, S_thresh[0], V_thresh[0]])
            red_upper = np.array([H_thresh, S_thresh[1], V_thresh[1]])
            mask_right = cv2.inRange(hsv_frame, red_lower, red_upper)

            # left mask H
            red_lower = np.array([180 - H_thresh, S_thresh[0], V_thresh[0]])
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

                frames["out_frame"] = cv2.rectangle(
                    img=frames["out_frame"],
                    pt1=(x, y),
                    pt2=(x+w, y+h),
                    color=config["display"]["line_color"],
                    thickness=2,
                    lineType=config["display"]["line_type"]
                )
                frames["out_frame"] = cv2.putText(
                    img=frames["out_frame"],
                    text=f"X: {x}, Y: {y}",
                    org=(x, y-10),
                    fontFace=config["display"]["font"],
                    fontScale=1,
                    color=config["display"]["line_color"],
                    thickness=1,
                    lineType=config["display"]["line_type"]
                )

            fps_handler.update()
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
                np.expand_dims(proc_frame, axis=2),
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

            if frame_idx >= wait_frames:
                annotations.append(
                    [f"{frame_idx - wait_frames:04}.jpg", image_size[1], image_size[0], "orb", x, y, x2, y2])
                cv2.imwrite(
                    f"./datasets/dataset_05/{frame_idx - wait_frames:04}.jpg",
                    raw_frame
                )

            if cv2.waitKey(1) == ord('q'):
                break

        df = pd.DataFrame(
            annotations,
            columns=["filename", "width", "height", "class", "xmin", "ymin", "xmax", "ymax"])
        df.to_csv("./datasets/dataset_05.csv", index=False)
