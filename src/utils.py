import os

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

    out_path_simple = os.path.join(models_dir, f"{out_filename}_simplified.onnx")
    onnx_model = onnx.load(out_path)
    model_simple, check = simplify(onnx_model)
    onnx.save(model_simple, out_path_simple)

    return out_path_simple
