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


def draw_box(frame: npt.NDArray, bbox: tuple, CONFIG: dict, color=None):
    x, y, x2, y2 = bbox
    return cv2.rectangle(
        img=frame,
        pt1=(x, y),
        pt2=(x2, y2),
        color=CONFIG["display"]["line_color"] if not color else color,
        thickness=CONFIG["display"]["line_thickness"]
    )
