import struct
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import random

'''
Classes:
0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite',
34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard',
37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 40: 'wine glass',
41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana',
47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot',
52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair',
57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table',
61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote',
66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven',
70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock',
75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier',
79: 'toothbrush'}
'''

def on_depth(msg: CompressedImage):
    # Parse format, e.g. "32FC1; compressedDepth png"
    parts = [p.strip() for p in msg.format.split(';')]
    img_encoding = parts[0]            # "16UC1" or "32FC1"
    compression = "png"
    for p in parts[1:]:
        if "compressedDepth rvl" in p:
            compression = "rvl"
        elif "compressedDepth png" in p or "compressedDepth" in p:
            compression = "png"

    if compression != "png":
        # RVL isn't a PNG — OpenCV can't decode it. Use republish or an RVL decoder.
        raise RuntimeError("compressedDepth is RVL-encoded. Use image_transport republish or RVL decoder.")

    # ---- 1) Read and skip the custom header (enum + 2 floats) ----
    header_fmt = "=i2f"  # int32 (format enum), float depthA, float depthB
    header_size = struct.calcsize(header_fmt)
    fmt_enum, depthA, depthB = struct.unpack_from(header_fmt, msg.data, 0)

    # ---- 2) PNG bytes start right after the header ----
    png_buf = np.frombuffer(msg.data, dtype=np.uint8, count=len(msg.data) - header_size, offset=header_size)

    # ---- 3) Decode PNG ----
    dec = cv2.imdecode(png_buf, cv2.IMREAD_UNCHANGED)
    if dec is None:
        raise RuntimeError("cv2.imdecode failed. Check header slicing and that format is PNG.")

    # ---- 4) Convert to depth in meters ----
    if img_encoding.startswith("32FC1"):
        # For 32F sources, the codec stored quantized *inverse depth* in a 16UC1 PNG.
        inv = dec.astype(np.float32)
        with np.errstate(divide='ignore', invalid='ignore'):
            depth_m = np.where(inv > 0, depthA / (inv - depthB), np.nan).astype(np.float32)
    elif img_encoding.startswith("16UC1"):
        # For 16U sources, this is raw depth in millimeters in the PNG.
        depth_m = dec.astype(np.float32) / 1000.0
    else:
        raise RuntimeError(f"Unexpected image encoding: {img_encoding}")
    
    return depth_m

def generate_random_colors(length, fix_seed = False):
    colors = []

    if fix_seed:
        random.seed(69)

    for i in range(0, length):
        colors.append((random.randint(0,255),random.randint(0,255),random.randint(0,255)))

    return colors

def srgb_to_linear(c):
    c = c/255.0
    return np.where(c <= 0.04045, c/12.92, ((c+0.055)/1.055)**2.4)

def best_text_color(bg):  # bg = (B,G,R) or (R,G,B) -> treat as RGB below
    r, g, b = bg  # if you have BGR, swap: r,g,b = bg[2],bg[1],bg[0]
    R, G, B = srgb_to_linear(np.array([r, g, b], dtype=float))
    L = 0.2126*R + 0.7152*G + 0.0722*B          # relative luminance
    # Contrast ratio vs black (L=0) and white (L=1)
    contrast_black = (L + 0.05) / 0.05
    contrast_white = 1.05 / (L + 0.05)
    return (0, 0, 0) if contrast_black >= contrast_white else (255, 255, 255)

def decode_class(cls):
    classes_dict = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
                    5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
                    10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
                    14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
                    20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
                    25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
                    30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite',
                    34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard',
                    37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 40: 'wine glass',
                    41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana',
                    47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot',
                    52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair',
                    57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table',
                    61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote',
                    66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven',
                    70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock',
                    75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier',
                    79: 'toothbrush'}
    
    return classes_dict[cls]