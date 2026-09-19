"""
Offload ResNet18 layer1[0].conv1 + bn1 + relu (64 -> 64 channels, 56x56) of the
melanoma classifier to the Eyeriss ECP5 accelerator.

    stem (CPU, float) -> [FPGA: 3x3 conv, BN folded into weight/bias, ReLU] -> rest (CPU, float)

Checks
    1. FPGA output == bit-accurate integer golden model
    2. SQNR of the dequantized FPGA output vs the float layer
    3. Classification with the FPGA layer vs the float model

Usage (run from the model project, which provides ./model and ./sample_images):
    python deploy_resnet_eyeriss.py [COM3] [--emulate] [--image path]
"""
import argparse
import glob
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from torchvision import models, transforms

EYERISS_DIR = os.environ.get(
    "EYERISS_DIR", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, EYERISS_DIR)
import eyeriss_host as eh  # noqa: E402

MODEL_PATH = "./model/resnet18_melanoma.pth"
IMAGE_GLOB = "./sample_images/*.jpg"
CLASSES = ["benign", "malignant"]      # ImageFolder order used by train_dl.py

# Activation / output clipping percentiles (saturation is part of the golden model)
ACT_PERCENTILE = 99.9
OUT_PERCENTILE = 99.9

EVAL_TRANSFORM = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])


# ==========================================================================
# Model helpers
# ==========================================================================
def load_model():
    model = models.resnet18()
    model.fc = nn.Linear(model.fc.in_features, 2)
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"{MODEL_PATH} not found, run train_dl.py first")
    model.load_state_dict(torch.load(MODEL_PATH, map_location="cpu", weights_only=True))
    return model.eval()


def stem(model, x):
    return model.maxpool(model.relu(model.bn1(model.conv1(x))))


def finish_from_block0(model, x_in, block0_mid):
    """Rest of the network, given relu(bn1(conv1(x))) of layer1[0]."""
    blk = model.layer1[0]
    out = blk.bn2(blk.conv2(block0_mid))
    out = F.relu(out + x_in)                      # layer1[0] has no downsample
    out = model.layer1[1](out)
    out = model.layer4(model.layer3(model.layer2(out)))
    out = torch.flatten(model.avgpool(out), 1)
    return model.fc(out)


def fold_bn(conv, bn):
    std = torch.sqrt(bn.running_var + bn.eps)
    w = conv.weight * (bn.weight / std)[:, None, None, None]
    b = bn.bias - bn.running_mean * bn.weight / std
    if conv.bias is not None:
        b = b + conv.bias * bn.weight / std
    return w.detach().numpy().astype(np.float64), b.detach().numpy().astype(np.float64)


# ==========================================================================
# Quantization
# ==========================================================================
def quantize_activation(x):
    """Non-negative float -> uint8 with zero point 128 (7-bit magnitude)."""
    s_x = float(np.percentile(x, ACT_PERCENTILE)) / 127.0
    q = np.clip(np.round(x / s_x), 0, 127).astype(np.int64) + 128
    return q.astype(np.uint8), s_x


def quantize_layer(w_f, b_f, s_x):
    s_w = float(np.max(np.abs(w_f))) / 127.0
    w_q = np.clip(np.round(w_f / s_w), -128, 127).astype(np.int8)
    b_q = np.round(b_f / (s_x * s_w)).astype(np.int64)
    return w_q, b_q, s_w


def pick_output_shift(psum):
    """Right shift that maps the upper percentile of positive psums to 127."""
    peak = float(np.percentile(psum[psum > 0], OUT_PERCENTILE)) if np.any(psum > 0) else 1.0
    shift = 0
    while peak / (1 << shift) > 127 and shift < 31:
        shift += 1
    return shift


# ==========================================================================
# Main
# ==========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM3")
    ap.add_argument("--emulate", action="store_true", help="use the software model")
    ap.add_argument("--image", default=None)
    args = ap.parse_args()

    image_path = args.image or sorted(glob.glob(IMAGE_GLOB))[0]
    print(f"[1] Model & image: {os.path.basename(image_path)}")
    model = load_model()
    img = EVAL_TRANSFORM(Image.open(image_path).convert("RGB"))[None]

    with torch.no_grad():
        logits_ref = model(img)
        x_in = stem(model, img)                                   # 1,64,56,56
        blk = model.layer1[0]
        y_ref = F.relu(blk.bn1(blk.conv1(x_in)))[0].numpy()       # 64,56,56

    x_f = x_in[0].numpy().astype(np.float64)
    w_f, b_f = fold_bn(blk.conv1, blk.bn1)
    x_q, s_x = quantize_activation(x_f)
    w_q, b_q, s_w = quantize_layer(w_f, b_f, s_x)
    psum = eh.conv_psum(eh.pad_zero_point(x_q, 1), w_q, b_q)
    shift = pick_output_shift(psum)
    s_y = s_x * s_w * (1 << shift)
    golden = eh.ppu(psum, shift, relu=True)
    print(f"    ifmap {tuple(x_q.shape)}  weight {tuple(w_q.shape)}  "
          f"s_x={s_x:.4g} s_w={s_w:.4g} shift={shift}")

    print(f"\n[2] FPGA inference ({'emulator' if args.emulate else args.port})")
    try:
        transport = eh.EmulatedDevice() if args.emulate else eh.SerialTransport(args.port, timeout=10)
        acc = eh.Eyeriss(transport)
        acc.ping()
    except Exception as err:
        print(f"[!] {err}")
        return 1

    def progress(done, total):
        print(f"\r    tile {done}/{total}", end="", flush=True)

    t0 = time.perf_counter()
    try:
        y_hw = acc.conv_layer(x_q, w_q, b_q, shift, relu=True, padding=1, progress=progress)
    finally:
        acc.close()
    dt = time.perf_counter() - t0
    g = -(-x_q.shape[0] // eh.CH_PER_WORD)
    print(f"\n    {y_hw.size} outputs in {dt:.2f} s "
          f"(tile {eh.max_tile_width(g, eh.MAX_P)}x{eh.max_tile_width(g, eh.MAX_P)}, "
          f"{y_hw.size / dt:.0f} px/s)")

    print("\n[3] Verification")
    n_ok = int(np.sum(y_hw == golden))
    exact = n_ok == golden.size
    print(f"    FPGA vs integer golden : {n_ok} / {golden.size} match")

    y_deq = (y_hw.astype(np.float64) - 128) * s_y
    err = y_ref - y_deq
    sqnr = 10 * np.log10(np.sum(y_ref ** 2) / max(np.sum(err ** 2), 1e-30))
    cos = float(np.sum(y_ref * y_deq) / (np.linalg.norm(y_ref) * np.linalg.norm(y_deq) + 1e-30))
    print(f"    FPGA vs float layer    : SQNR {sqnr:.1f} dB, cosine {cos:.4f}")

    with torch.no_grad():
        logits_hw = finish_from_block0(model, x_in, torch.tensor(y_deq, dtype=torch.float32)[None])
    p_ref = torch.softmax(logits_ref, 1)[0].numpy()
    p_hw = torch.softmax(logits_hw, 1)[0].numpy()
    same = int(np.argmax(p_ref)) == int(np.argmax(p_hw))
    print(f"    float model            : {CLASSES[int(np.argmax(p_ref))]:9s} p={np.round(p_ref, 4)}")
    print(f"    with FPGA layer        : {CLASSES[int(np.argmax(p_hw))]:9s} p={np.round(p_hw, 4)}")

    if exact and same:
        print("\n[OK] FPGA layer is bit-exact and the prediction is unchanged")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
