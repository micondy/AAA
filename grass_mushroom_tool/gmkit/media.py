"""图像落盘与缩略图。

只依赖标准库；如果装了 Pillow / OpenCV 会自动用上（更快的缩放与 EXIF 方向修正）。
"""

from __future__ import annotations

import io
import struct
import zlib
from pathlib import Path

try:  # pragma: no cover - 取决于运行环境
    from PIL import Image, ImageOps

    _PIL = True
except Exception:  # pragma: no cover
    Image = ImageOps = None
    _PIL = False

JPEG_MAGIC = b"\xff\xd8"
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def has_pillow() -> bool:
    return _PIL


def guess_ext(data: bytes, default: str = ".jpg") -> str:
    """按文件头推断扩展名，避免把 PNG 存成 .jpg。"""
    if data.startswith(JPEG_MAGIC):
        return ".jpg"
    if data.startswith(PNG_MAGIC):
        return ".png"
    if data[:4] == b"RIFF" and data[8:12] == b"WEBP":
        return ".webp"
    if data[:6] in (b"GIF87a", b"GIF89a"):
        return ".gif"
    return default


def probe_size(data_or_path) -> tuple:
    """返回 (宽, 高)，失败时返回 (None, None)。"""
    if not _PIL:
        return None, None
    try:
        source = io.BytesIO(data_or_path) if isinstance(data_or_path, (bytes, bytearray)) else str(data_or_path)
        with Image.open(source) as image:
            return image.size
    except Exception:
        return None, None


def write_png(path: Path, array) -> None:
    """把 numpy 数组写成 PNG（uint8 RGB / uint16 灰度）。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if _PIL:
        try:
            image = Image.fromarray(array)
            image.save(path, format="PNG", compress_level=3)
            return
        except Exception:
            pass
    path.write_bytes(encode_png(array))


def encode_png(array) -> bytes:
    """不依赖 Pillow 的最小 PNG 编码器。"""
    import numpy as np

    array = np.ascontiguousarray(array)
    height, width = array.shape[:2]
    if array.ndim == 3:
        channels = array.shape[2]
        if channels not in (3, 4):
            raise ValueError(f"不支持的通道数: {channels}")
        color_type = 2 if channels == 3 else 6
        bit_depth = 8
        raw = array.astype(np.uint8, copy=False).tobytes()
        pixel_bytes = channels
    else:
        color_type = 0
        if array.dtype == np.uint16:
            bit_depth = 16
            raw = array.astype(">u2", copy=False).tobytes()
            pixel_bytes = 2
        else:
            bit_depth = 8
            raw = array.astype(np.uint8, copy=False).tobytes()
            pixel_bytes = 1

    stride = width * pixel_bytes
    rows = bytearray()
    for y in range(height):
        rows.append(0)  # filter type 0
        rows += raw[y * stride:(y + 1) * stride]

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + tag
            + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    header = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    return (
        PNG_MAGIC
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(bytes(rows), 6))
        + chunk(b"IEND", b"")
    )


_DEPTH_COLORS = None


def depth_colormap():
    """256 级 jet 色表（蓝→青→绿→黄→红）。"""
    global _DEPTH_COLORS
    if _DEPTH_COLORS is None:
        import numpy as np

        t = np.linspace(0.0, 1.0, 256)
        red = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
        green = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
        blue = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
        _DEPTH_COLORS = (np.stack([red, green, blue], axis=1) * 255).astype(np.uint8)
    return _DEPTH_COLORS


def colorize_depth(depth, low=None, high=None):
    """深度数组（uint16 mm）-> RGB：近处偏红、远处偏蓝、无效值涂黑。

    ``low``/``high`` 省略时按当前场景 2%–98% 分位自动拉伸。
    """
    import numpy as np

    depth = np.asarray(depth)
    if low is None or high is None:
        valid = depth[depth > 0]
        if valid.size >= max(64, depth.size * 0.05):
            low = float(np.percentile(valid, 2))
            high = float(np.percentile(valid, 98))
        else:
            low, high = 150.0, 3000.0
    if high - low < 200:
        high = low + 200
    span = float(high) - float(low)
    norm = 1.0 - np.clip((depth.astype("float32") - low) / span, 0.0, 1.0)
    rgb = depth_colormap()[(norm * 255).astype(np.uint8)]
    rgb[depth == 0] = 0
    return rgb


def colorize_png(source: Path, target: Path, width: int = 720, quality: int = 88) -> bool:
    """把 16 位深度 PNG 转成可读的伪彩 JPEG（缩略图/大图预览用）。"""
    if not _PIL:
        return False
    try:
        import numpy as np

        with Image.open(source) as image:
            depth = np.array(image)
        rgb = colorize_depth(depth)
        picture = Image.fromarray(rgb)
        if width and picture.width > width:
            ratio = width / float(picture.width)
            picture = picture.resize((width, max(1, round(picture.height * ratio))), Image.BILINEAR)
        target.parent.mkdir(parents=True, exist_ok=True)
        picture.save(target, format="JPEG", quality=int(quality), optimize=True)
        return True
    except Exception:
        return False


def to_jpeg(array, quality: int = 70, width: int = None) -> bytes:
    if not _PIL:
        raise RuntimeError("需要安装 Pillow 才能输出 JPEG 实时画面")
    import numpy as np

    image = Image.fromarray(np.ascontiguousarray(array))
    if width and image.width > width:
        ratio = width / float(image.width)
        image = image.resize((width, max(1, round(image.height * ratio))), Image.BILINEAR)
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG", quality=int(quality))
    return buffer.getvalue()


def solid_jpeg(width: int = 320, height: int = 180, color=(18, 28, 46), quality: int = 55) -> bytes:
    """一张纯色占位图，用于相机还没出图时保持 <img> 存活。"""
    if not _PIL:
        raise RuntimeError("需要安装 Pillow 才能输出 JPEG 实时画面")
    image = Image.new("RGB", (max(8, int(width)), max(8, int(height))), tuple(color))
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG", quality=quality)
    return buffer.getvalue()


def make_thumbnail(source: Path, target: Path, width: int = 360, quality: int = 82) -> bool:
    """生成缩略图缓存；没有 Pillow 时返回 False（由调用方回退到原图）。"""
    if _PIL and str(source).endswith("d435i-depth.png"):
        return colorize_png(source, target, width=width, quality=quality)
    if not _PIL:
        return False
    try:
        with Image.open(source) as image:
            image = ImageOps.exif_transpose(image)
            if image.mode not in ("RGB", "L"):
                image = image.convert("RGB")
            ratio = width / float(image.width or 1)
            if ratio < 1:
                image = image.resize(
                    (width, max(1, round(image.height * ratio))), Image.LANCZOS
                )
            target.parent.mkdir(parents=True, exist_ok=True)
            image.save(target, format="JPEG", quality=quality, optimize=True)
        return True
    except Exception:
        return False
