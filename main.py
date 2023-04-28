import os
import time

import blobconverter
import cv2
import depthai as dai
import kornia
import numpy as np
import numpy.typing as npt
import onnx
import torch
from onnxsim import simplify
from torch import nn
from torchvision import transforms

models_dir = "./models"
# image_size = (480, 640)  # (height, width)
image_size = (720, 1280)  # (height, width)
font = cv2.FONT_HERSHEY_SIMPLEX

# class RedFilter(nn.Module):

#     def forward(self, X: torch.Tensor):
#         y = kornia.color.rgb_to_hsv(X / 255.0)

#         return kornia.color.hsv_to_rgb(y)

#     # def forward(self, X: torch.Tensor):
#     #     """_summary_

#     #     Args:
#     #         X (_type_): (channels, height, width)

#     #     Returns:
#     #         _type_: (height, width)
#     #     """
#     #     # y_r = X[0, :, :]
#     #     # y_g = X[1, :, :]
#     #     # y_b = X[2, :, :]

#     #     # y = y_r - 0.5 * y_g - 0.5 * y_b
#     #     # y = torch.maximum(y, torch.tensor(0, dtype=torch.float))

#     #     # zero_tensor = torch.zeros_like(X)

#     #     y = kornia.color.rgb_to_hsv(X / 255.0)
#     #     print(f"y.shape: {y.shape}")

#     #     output_rgb = kornia.color.hsv_to_rgb(y)[0, :, :]
#     #     print(f"output_rgb.shape: {output_rgb.shape}")

#     #     return output_rgb


def pytorch_to_onnx(model: nn.Module, input_shape: tuple, simplify_onnx: bool = True):
    output_file_name = model.__class__.__name__

    X = torch.ones(input_shape, dtype=torch.float)
    output_path = os.path.join(models_dir, f"{output_file_name}.onnx")
    torch.onnx.export(
        model,
        X,
        output_path,
        opset_version=14,
        do_constant_folding=True
    )

    if not simplify_onnx:
        return output_path

    output_path_simplified = os.path.join(models_dir, f"{output_file_name}_simplified.onnx")
    onnx_model = onnx.load(output_path)
    simplified_model, check = simplify(onnx_model)
    onnx.save(simplified_model, output_path_simplified)

    return output_path_simplified


X_shape = (3, *image_size)
# onnx_path = pytorch_to_onnx(RedFilter(), X_shape, simplify_onnx=False)

# blob_path = blobconverter.from_onnx(
#     model=onnx_path,
#     output_dir=models_dir,
#     data_type="FP16",
#     shaves=6,
#     use_cache=False,
#     optimizer_params=[]
# )

if __name__ == "__main__":
    # Create pipeline
    pipeline = dai.Pipeline()

    # RGB camera
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setInterleaved(False)
    camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camera.setPreviewSize(width=image_size[1], height=image_size[0])
    camera.setFps(60)

    # Red filter
    # red_filter = pipeline.create(dai.node.NeuralNetwork)
    # red_filter.setBlobPath(blob_path)
    # camera.preview.link(red_filter.input)

    # Create output
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    # red_filter.out.link(xout_rgb.input)
    camera.preview.link(xout_rgb.input)

    # Pipeline is defined, now we can connect to the device
    with dai.Device(pipeline, usb2Mode=True) as device:
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)  # type: ignore

        raw_frame: npt.NDArray = np.zeros_like((image_size[0], image_size[1], 3), dtype=np.uint8)

        # used to record the time when we processed last frame
        prev_frame_time = 0

        # used to record the time at which we processed current frame
        new_frame_time = 0

        # Main host-side application loop
        while True:
            # we try to fetch the data from nn/rgb queues. tryGet will return either the data packet or None if there isn't any
            in_rgb: dai.ImgFrame | None = q_rgb.tryGet()  # type: ignore

            if in_rgb is not None:
                raw_frame = in_rgb.getCvFrame()  # type: ignore

            # if in_rgb is not None:
            #     # If the packet from RGB camera is present, we're retrieving the frame in OpenCV format using getCvFrame
            #     raw_frame = in_rgb.getCvFrame()  # type: ignore
            #     assert raw_frame is not None
            #     img_hsv = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2HSV)

            #     # lower mask (0-10)
            #     lower_red = np.array([0, 100, 20])
            #     upper_red = np.array([10, 255, 255])
            #     mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

            #     # upper mask (170-180)
            #     lower_red = np.array([170, 100, 20])
            #     upper_red = np.array([180, 255, 255])
            #     mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

            #     mask = mask0 + mask1  # type: ignore

            #     # set my output img to zero everywhere except my mask
            #     output_rgb = raw_frame.copy()
            #     output_rgb[np.where(mask == 0)] = 0

            #     # or your HSV image, which I *believe* is what you want
            #     output_hsv = img_hsv.copy()
            #     output_hsv[np.where(mask == 0)] = 0
            #     # red_frame = frame[:, :, 0] - frame[:, :, 1] - frame[:, :, 2]
            #     # print(frame.shape)
            #     # red_frame = frame[:, :, 0]
            #     # red_frame = frame[:, :, 2]
            #     proc_frame = output_rgb
            #     proc_frame = cv2.blur(proc_frame, (30, 30))

            #     ret, proc_frame = cv2.threshold(proc_frame, 50, 255, type=cv2.THRESH_BINARY)
            #     proc_frame = cv2.cvtColor(proc_frame, cv2.COLOR_RGB2GRAY)

            #     contours, hierarchy = cv2.findContours(proc_frame, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

            #     if len(contours) > 0:
            #         cnt = contours[0]

            #         x, y, w, h = cv2.boundingRect(cnt)
            #         raw_frame = cv2.rectangle(raw_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            #         raw_frame = cv2.putText(
            #             raw_frame,
            #             f"X: {x}, Y: {y}",
            #             (x, y-10),
            #             font,
            #             0.75,
            #             (0, 255, 0),
            #             1,
            #             cv2.LINE_AA
            #         )
            #     # frame = np.where(red_frame > 100, 255, 0).astype(np.uint8)

            if raw_frame is not None:
                new_frame_time = time.time()
                # Calculating the fps

                # fps will be number of frame processed in given time frame
                # since their will be most of time error of 0.001 second
                # we will be subtracting it to get more accurate result
                fps = 1 / (new_frame_time-prev_frame_time)
                prev_frame_time = new_frame_time

                # converting the fps into integer
                fps = int(fps)

                # converting the fps to string so that we can display it on frame
                # by using putText function
                fps = str(fps)

                # putting the FPS count on the frame
                cv2.putText(
                    img=raw_frame,
                    text=fps,
                    org=(7, 70),
                    fontFace=font,
                    fontScale=1,
                    color=(100, 255, 0),
                    thickness=2,
                    lineType=cv2.LINE_AA
                )
                cv2.imshow("Custom Obect Tracker", raw_frame)

            # at any time, you can press "q" and exit the main loop, therefore exiting the program itself
            if cv2.waitKey(1) == ord('q'):
                break
