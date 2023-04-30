import time

import cv2
import depthai as dai
import numpy as np
import numpy.typing as npt

# Config
font = cv2.FONT_HERSHEY_SIMPLEX
line_type = cv2.LINE_AA
GREEN = (0, 255, 0)

image_size = (720, 1280)  # (height, width)
X_shape = (3, *image_size)


# Create pipeline
pipeline = dai.Pipeline()

# RGB camera
camera = pipeline.create(dai.node.ColorCamera)
camera.setInterleaved(False)
camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camera.setIspScale(2, 3)  # 1280x720
camera.setImageOrientation(dai.CameraImageOrientation.VERTICAL_FLIP)

# Create output
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
camera.isp.link(xout_rgb.input)

# Connect to the device
with dai.Device(pipeline, usb2Mode=True) as device:
    q_rgb: dai.DataOutputQueue = device.getOutputQueue(name="rgb", maxSize=4)  # type: ignore

    raw_frame: npt.NDArray = np.zeros_like((image_size[0], image_size[1], 3), dtype=np.uint8)

    prev_time: int = 0
    new_time: int = 0

    # Main host-side application loop
    while True:
        in_rgb: dai.ImgFrame = q_rgb.get()  # type: ignore

        raw_frame = in_rgb.getCvFrame()  # type: ignore

        
        hsv_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2HSV)

        # right mask H: (0-10)
        red_lower = np.array([0, 200, 20])
        red_upper = np.array([10, 255, 255])
        mask_right = cv2.inRange(hsv_frame, red_lower, red_upper)

        # left mask H: (170-180)
        red_lower = np.array([170, 200, 20])
        red_upper = np.array([180, 255, 255])
        mask_left = cv2.inRange(hsv_frame, red_lower, red_upper)

        mask = mask_right + mask_left  # type: ignore

        # Set output frame to zero everywhere except the mask
        output_rgb = raw_frame.copy()
        output_rgb[np.where(mask == 0)] = 0

        # Post processing
        proc_frame = output_rgb
        proc_frame = cv2.blur(proc_frame, (40, 40))
        ret, proc_frame = cv2.threshold(proc_frame, 50, 255, type=cv2.THRESH_BINARY)
        proc_frame = cv2.cvtColor(proc_frame, cv2.COLOR_RGB2GRAY)

        contours, hierarchy = cv2.findContours(
            image=proc_frame,
            mode=cv2.RETR_TREE,
            method=cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) > 0:
            cnt = contours[0]
            x, y, w, h = cv2.boundingRect(cnt)

            raw_frame = cv2.rectangle(
                img=raw_frame,
                pt1=(x, y),
                pt2=(x+w, y+h),
                color=GREEN,
                thickness=2,
                lineType=line_type
            )
            raw_frame = cv2.putText(
                img=raw_frame,
                text=f"X: {x}, Y: {y}",
                org=(x, y-10),
                fontFace=font,
                fontScale=0.75,
                color=GREEN,
                thickness=1,
                lineType=line_type
            )

        if raw_frame is not None:
            # FPS counter
            new_time = time.time_ns()
            fps = 1e9 / (new_time - prev_time)
            prev_time = new_time

            # putting the FPS count on the frame
            cv2.putText(
                img=raw_frame,
                text=f"FPS: {fps:.2f}",
                org=(5, 30),
                fontFace=font,
                fontScale=1,
                color=(100, 255, 0),
                thickness=2,
                lineType=line_type
            )
            cv2.imshow("Custom Object Tracker", raw_frame)

        # at any time, you can press "q" and exit the main loop, therefore exiting the program itself
        if cv2.waitKey(1) == ord('q'):
            break
