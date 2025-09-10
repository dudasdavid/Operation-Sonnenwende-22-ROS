import struct
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

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