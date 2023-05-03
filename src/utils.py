import os

import cv2
import numpy as np
import numpy.typing as npt
import onnx
import torch
from onnxsim import simplify
from torch import nn

models_dir = "./models"


def pytorch_to_onnx(model: nn.Module, input_shape: tuple, simplify_onnx: bool = True):
    out_filename = model.__class__.__name__

    X = torch.ones(input_shape, dtype=torch.float32)
    out_path = os.path.join(models_dir, f"{out_filename}.onnx")
    torch.onnx.export(
        model=model,
        args=X,
        f=out_path,
        opset_version=12,
        do_constant_folding=True
    )

    if not simplify_onnx:
        return out_path

    out_path_simple = os.path.join(
        models_dir, f"{out_filename}_simplified.onnx")
    onnx_model = onnx.load(out_path)
    model_simple, check = simplify(onnx_model)
    onnx.save(model_simple, out_path_simple)

    return out_path_simple


def cv2_put_xyz(frame, xyz, anchor, CONFIG):
    x, y, z = xyz[0], xyz[1], xyz[2]
    ax, ay = anchor
    cv2.putText(
        img=frame,
        text=f"X: {x} mm",
        org=(ax + 10, ay + 20),
        fontFace=CONFIG["display"]["font"],
        fontScale=0.5,
        color=CONFIG["display"]["line_color"],
        lineType=CONFIG["display"]["line_type"]
    )
    cv2.putText(
        img=frame,
        text=f"Y: {y} mm",
        org=(ax + 10, ay + 35),
        fontFace=CONFIG["display"]["font"],
        fontScale=0.5,
        color=CONFIG["display"]["line_color"],
        lineType=CONFIG["display"]["line_type"]
    )
    cv2.putText(
        img=frame,
        text=f"Z: {z} mm",
        org=(ax + 10, ay + 50),
        fontFace=CONFIG["display"]["font"],
        fontScale=0.5,
        color=CONFIG["display"]["line_color"],
        lineType=CONFIG["display"]["line_type"]
    )


def cnt_centroid(cnt):
    M = cv2.moments(cnt)
    if M['m00'] == 0:
        return None

    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    centroid = np.array([[cX, cY]])
    return centroid


def circle_like(cnt):
    centroid = cnt_centroid(cnt)
    if centroid is None:
        return 0

    r_hat = np.linalg.norm(cnt[:, 0, :] - centroid, axis=1)
    _, r_ref = cv2.minEnclosingCircle(cnt)

    rms_error = np.sqrt(np.mean((r_ref - r_hat) ** 2))
    return rms_error


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
    _, frame = cv2.threshold(
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
    contours, _ = cv2.findContours(
        image=frame,
        mode=cv2.RETR_LIST,
        method=cv2.CHAIN_APPROX_NONE
    )

    if len(contours) >= 2:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

        # index 0 is always ORB, index 1 is always CAR
        contours = sorted(contours, key=circle_like)

        cnt_orb = contours[0]
        orb_x, orb_y, orb_w, orb_h = cv2.boundingRect(cnt_orb)
        orb_x2, orb_y2 = orb_x + orb_w, orb_y + orb_h
        bb_orb = (orb_x, orb_y, orb_x2, orb_y2)

        cnt_car = contours[1]
        car_x, car_y, car_w, car_h = cv2.boundingRect(cnt_car)
        car_x2, car_y2 = car_x + car_w, car_y + car_h
        bb_car = (car_x, car_y, car_x2, car_y2)

        return bb_orb, bb_car

    return None


def draw_box(frame: npt.NDArray, bbox: tuple, CONFIG: dict, color=None):
    x, y, x2, y2 = bbox
    return cv2.rectangle(
        img=frame,
        pt1=(x, y),
        pt2=(x2, y2),
        color=CONFIG["display"]["line_color"] if not color else color,
        thickness=CONFIG["display"]["line_thickness"]
    )
