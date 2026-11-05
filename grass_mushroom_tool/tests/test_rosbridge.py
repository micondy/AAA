"""D435i 桥接的纯逻辑测试（不需要真实相机）。

重点验证：彩色/深度像素对齐后的数组解码、时间戳配对、meta 记录。
"""

import numpy as np
import pytest

from gmkit import naming
from gmkit.rosbridge import RosBridge, RosError, _image_array, _meta, _stamp_seconds


class Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class Header:
    def __init__(self, sec=0, nanosec=0, frame_id="camera_color_optical_frame"):
        self.stamp = Stamp(sec, nanosec)
        self.frame_id = frame_id


class Info:
    def __init__(self, width=848, height=480):
        self.header = Header(sec=1)
        self.width = width
        self.height = height
        self.k = [600.0, 0.0, 424.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        self.d = [0.0] * 5


class Image:
    def __init__(self, width, height, encoding, data, step=None, sec=1, frame_id="camera_color_optical_frame"):
        self.header = Header(sec=sec, frame_id=frame_id)
        self.width = width
        self.height = height
        self.encoding = encoding
        self.data = data
        if step is not None:
            self.step = step
        elif encoding in ("rgb8", "bgr8"):
            self.step = width * 3
        elif encoding in ("rgba8", "bgra8"):
            self.step = width * 4
        else:
            self.step = width * 2


def rgb8_image(sec=1):
    # 3x2，每行有 2 字节填充（step = 11）
    row = bytes([1, 2, 3, 4, 5, 6, 7, 8, 9]) + b"\x00\x00"
    return Image(3, 2, "rgb8", row * 2, step=11, sec=sec)


def depth16_image(sec=1, values=None):
    values = values or [[100, 200, 300], [400, 500, 600]]
    rows = b""
    for line in values:
        rows += np.asarray(line + [0], dtype="<u2").tobytes()  # 尾部 1 个填充像素
    return Image(3, 2, "16UC1", rows, step=8, sec=sec, frame_id="camera_color_optical_frame")


# ---------------------------------------------------------------------------
# 图像解码
# ---------------------------------------------------------------------------


def test_rgb8_decoding_respects_row_padding():
    array = _image_array(rgb8_image())
    assert array.shape == (2, 3, 3)
    assert array[0, 0].tolist() == [1, 2, 3]
    assert array[1, 2].tolist() == [7, 8, 9]


def test_bgr8_is_swapped_to_rgb():
    row = bytes([10, 20, 30, 40, 50, 60, 70, 80, 90])
    array = _image_array(Image(3, 1, "bgr8", row))
    assert array[0, 0].tolist() == [30, 20, 10]


def test_depth16_decoding_keeps_millimetres():
    array = _image_array(depth16_image())
    assert array.dtype == np.uint16
    assert array.shape == (2, 3)
    assert array[0].tolist() == [100, 200, 300]
    assert array[1].tolist() == [400, 500, 600]


def test_unknown_encoding_is_rejected():
    with pytest.raises(RosError):
        _image_array(Image(2, 2, "yuv422", b"\x00" * 8))


# ---------------------------------------------------------------------------
# 时间戳配对
# ---------------------------------------------------------------------------


def test_stamp_seconds_handles_missing_stamp():
    assert _stamp_seconds(Image(1, 1, "16UC1", b"\x00\x00", sec=0)) is None
    assert _stamp_seconds(Image(1, 1, "16UC1", b"\x00\x00", sec=123)) == pytest.approx(123.0)


def test_pick_depth_prefers_aligned_frame():
    bridge = RosBridge("/cam")
    color = rgb8_image(sec=100)
    bridge._depth_buffer.append((depth16_image(sec=100), 0.0, False))  # 原始深度，正好同步
    bridge._depth_buffer.append((depth16_image(sec=100, values=[[1, 1, 1]] * 2), 0.0, True))

    picked, delta = bridge._pick_depth(color)
    assert picked is not None
    assert picked[2] is True  # 优先对齐深度
    assert delta == pytest.approx(0.0, abs=1e-6)


def test_pick_depth_rejects_frame_outside_tolerance():
    bridge = RosBridge("/cam")
    color = rgb8_image(sec=100)
    bridge._depth_buffer.append((depth16_image(sec=100, values=[[1, 1, 1]] * 2), 0.0, True))
    bridge._depth_buffer.append((depth16_image(sec=100.5), 0.0, True))

    picked, delta = bridge._pick_depth(color)
    assert picked is not None and delta == pytest.approx(0.0)  # 有匹配的那帧

    bridge2 = RosBridge("/cam")
    bridge2._depth_buffer.append((depth16_image(sec=100.5), 0.0, True))
    picked, delta = bridge2._pick_depth(color)
    assert picked is None
    assert delta == pytest.approx(0.5)


def test_pick_depth_rejects_raw_even_when_timestamp_matches():
    bridge = RosBridge("/cam")
    color = rgb8_image(sec=100)
    bridge._depth_buffer.append((depth16_image(sec=100), 0.0, False))
    picked, delta = bridge._pick_depth(color)
    assert picked is None and delta is None


def test_raw_frames_cannot_evict_aligned_depth():
    bridge = RosBridge("/cam")
    bridge._keep_depth(depth16_image(sec=100), True)
    for _ in range(20):
        bridge._keep_depth(depth16_image(sec=100), False)
    picked, delta = bridge._pick_depth(rgb8_image(sec=100))
    assert picked is not None and picked[2] is True
    assert delta == 0
    bridge._avail = True
    status = bridge.status()
    assert status["aligned"] is True
    assert status["age"]["raw_depth"] is not None


def test_pick_depth_returns_none_without_frames():
    bridge = RosBridge("/cam")
    picked, delta = bridge._pick_depth(rgb8_image())
    assert picked is None and delta is None


# ---------------------------------------------------------------------------
# meta 记录
# ---------------------------------------------------------------------------


def test_meta_records_alignment_and_intrinsics():
    bridge = RosBridge("/sensors/nav_camera")
    payload = _meta(
        color=rgb8_image(sec=100),
        depth=depth16_image(sec=100),
        aligned=True,
        delta=0.002,
        color_info=Info(),
        depth_info=Info(),
        cloud=None,
        points_written=0,
        stride=2,
        bridge=bridge,
    )
    assert payload["alignment"]["aligned_to_color"] is True
    assert payload["alignment"]["dt_ms"] == pytest.approx(2.0)
    assert payload["depth"]["topic"] == "/sensors/nav_camera/aligned_depth_to_color/image_raw"
    assert payload["depth"]["unit"] == "mm"
    assert payload["depth"]["scale_mm"] == 1
    assert payload["color"]["encoding"] == "rgb8"
    assert payload["color_camera_info"]["k"][0] == pytest.approx(600.0)
    assert payload["pointcloud"]["points_written"] == 0


def test_meta_marks_unaligned_fallback():
    bridge = RosBridge("/sensors/nav_camera")
    payload = _meta(
        color=rgb8_image(),
        depth=depth16_image(),
        aligned=False,
        delta=0.0,
        color_info=None,
        depth_info=None,
        cloud=None,
        points_written=0,
        stride=2,
        bridge=bridge,
    )
    assert payload["alignment"]["aligned_to_color"] is False
    assert payload["depth"]["topic"] == "/sensors/nav_camera/depth/image_rect_raw"
    assert payload["color_camera_info"] is None


def test_bridge_topics_follow_prefix():
    bridge = RosBridge("/camera/")
    assert bridge.color_topic == "/camera/color/image_raw"
    assert bridge.aligned_depth_topic == "/camera/aligned_depth_to_color/image_raw"
    assert bridge.raw_depth_topic == "/camera/depth/image_rect_raw"
    assert bridge.cloud_topic == "/camera/depth/color/points"
    assert bridge.depth_topic == bridge.aligned_depth_topic  # 兼容旧字段


def test_d435i_file_names_cover_alignment_variants():
    assert naming.d435i_file("S001", "depth") == "s001-d435i-depth.png"
    assert naming.d435i_file("S001", "points") == "s001-d435i-points.ply"


# ---------------------------------------------------------------------------
# 手机取景器（MJPEG 预览）
# ---------------------------------------------------------------------------


def test_colorize_depth_near_is_red_far_is_blue_hole_is_black():
    from gmkit.rosbridge import _colorize_depth

    depth = np.array([[0, 150, 3000]], dtype=np.uint16)
    rgb = _colorize_depth(depth, 150, 3000)
    assert rgb.shape == (1, 3, 3)
    assert rgb[0, 0].tolist() == [0, 0, 0]
    near = rgb[0, 1].astype(int)
    far = rgb[0, 2].astype(int)
    assert near[0] > near[2]  # 近处偏红
    assert far[2] > far[0]  # 远处偏蓝


def test_preview_returns_jpeg_for_each_mode():
    bridge = RosBridge("/cam")
    assert bridge.preview()[0] is None  # 还没有画面

    bridge._keep("color", rgb8_image(sec=100))
    bridge._keep_depth(depth16_image(sec=100), True)

    for mode in ("color", "depth", "rgbd"):
        payload, seq = bridge.preview(mode=mode, width=96, quality=60)
        assert payload is not None
        assert payload[:2] == b"\xff\xd8"  # JPEG SOI
        assert seq == bridge.frame_seq() > 0


def test_preview_ignores_stale_frames():
    import time

    bridge = RosBridge("/cam")
    bridge._latest["color"] = (rgb8_image(sec=100), time.monotonic() - 60)
    payload, _seq = bridge.preview(mode="color")
    assert payload is None


def test_media_jpeg_helpers():
    from gmkit import media

    payload = media.solid_jpeg(64, 36)
    assert payload[:2] == b"\xff\xd8"
    rgb = np.zeros((36, 64, 3), dtype=np.uint8)
    rgb[:, :, 1] = 200
    assert media.to_jpeg(rgb, quality=70, width=32)[:2] == b"\xff\xd8"


# ---------------------------------------------------------------------------
# 快照落盘（用假消息，不需要相机）
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("problem", ["raw", "stale", "size", "frame"])
def test_snapshot_rejects_invalid_depth_without_writing(tmp_path, monkeypatch, problem):
    import time

    bridge = RosBridge("/cam")
    bridge._avail = True
    monkeypatch.setattr(bridge, "start", lambda: True)
    pair = bridge._pair
    monkeypatch.setattr(bridge, "_pair", lambda: pair(timeout=0))
    bridge._keep("color", rgb8_image(sec=100))
    depth = depth16_image(sec=100)
    if problem == "size":
        depth.width = 2
    if problem == "frame":
        depth.header.frame_id = "camera_depth_optical_frame"
    bridge._keep_depth(depth, problem != "raw")
    if problem == "stale":
        bridge._depth_buffer.clear()
        bridge._depth_buffer.append((depth, time.monotonic() - 6, True))
    with pytest.raises(RosError, match="未保存快照"):
        bridge.snapshot(tmp_path, "S001", want_points=False)
    assert list(tmp_path.iterdir()) == []


def test_preview_can_show_raw_but_snapshot_pair_cannot_use_it():
    bridge = RosBridge("/cam")
    bridge._keep("color", rgb8_image(sec=100))
    bridge._keep_depth(depth16_image(sec=100), False)
    assert bridge.preview()[0].startswith(b"\xff\xd8")
    assert bridge._pair(timeout=0)[1] is None


def test_snapshot_writes_paired_rgbd_and_registers(tmp_path, monkeypatch):
    import json
    import time

    from gmkit import store as store_mod

    store = store_mod.Store(tmp_path / "data")
    session = store.create(date="20261105", rooms="A,2,2")
    store.add_sample(room="A")

    bridge = RosBridge("/cam")
    bridge._avail = True
    monkeypatch.setattr(bridge, "start", lambda: True)
    now = time.monotonic()
    bridge._latest["color"] = (rgb8_image(sec=100), now)
    bridge._latest["color_info"] = (Info(3, 2), now)
    bridge._depth_buffer.append((depth16_image(sec=100), now, True))

    files = bridge.snapshot(session.root, "S001", want_points=False)
    kinds = {item["kind"] for item in files}
    assert kinds == {"color", "depth", "meta"}

    color_path = session.root / naming.d435i_rel("S001", "color")
    depth_path = session.root / naming.d435i_rel("S001", "depth")
    assert color_path.is_file() and depth_path.is_file()
    assert color_path.name == "s001-d435i-color.png"
    assert depth_path.name == "s001-d435i-depth.png"

    # 深度必须是 16 位、且与彩色尺寸一致（说明对齐后逐像素可用）
    from PIL import Image as PILImage

    with PILImage.open(color_path) as rgb, PILImage.open(depth_path) as dep:
        assert rgb.size == dep.size == (3, 2)
        assert dep.mode in ("I;16", "I", "I;16B")

    meta = json.loads((session.root / naming.d435i_rel("S001", "meta")).read_text(encoding="utf-8"))
    assert meta["alignment"]["aligned_to_color"] is True
    assert meta["depth"]["unit"] == "mm"
    assert meta["color"]["width"] == 3

    entries = store.add_sensor_files("S001", files)
    assert len(entries) == 3
    slots = {p["slot"] for p in store.require().get_sample("S001")["photos"]}
    assert slots == {"d435i-color", "d435i-depth", "d435i-meta"}


def test_auto_depth_range_stretches_to_scene():
    from gmkit.rosbridge import _auto_depth_range

    depth = np.zeros((20, 20), dtype=np.uint16)
    depth[2:18, 2:18] = np.linspace(1000, 1400, 256).astype(np.uint16).reshape(16, 16)
    low, high = _auto_depth_range(depth)
    assert 950 <= low <= 1100
    assert 1300 <= high <= 1450


def test_auto_depth_range_falls_back_without_valid_depth():
    from gmkit.rosbridge import _auto_depth_range

    assert _auto_depth_range(np.zeros((20, 20), dtype=np.uint16)) == (150, 3000)


def test_d435i_group_sequence_names():
    assert naming.d435i_file("S001", "color", seq=1) == "s001-d435i-color.png"
    assert naming.d435i_file("S001", "color", seq=2) == "s001-d435i-color-2.png"
    assert naming.d435i_rel("S007", "points", seq=3) == "samples/S007/s007-d435i-points-3.ply"


def test_multiple_rgbd_groups_do_not_overwrite(tmp_path, monkeypatch):
    import time

    from gmkit import store as store_mod

    store = store_mod.Store(tmp_path / "data")
    session = store.create(date="20261105")
    store.add_sample(room="A")

    bridge = RosBridge("/cam")
    bridge._avail = True
    monkeypatch.setattr(bridge, "start", lambda: True)

    for group in (1, 2):
        now = time.monotonic()
        bridge._latest["color"] = (rgb8_image(sec=100 + group), now)
        bridge._latest["color_info"] = (Info(3, 2), now)
        bridge._depth_buffer.clear()
        bridge._depth_buffer.append((depth16_image(sec=100 + group), now, True))
        files = bridge.snapshot(session.root, "S001", want_points=False, seq=group)
        store.add_sensor_files("S001", files)

    names = sorted(p["name"] for p in store.require().get_sample("S001")["photos"])
    assert "s001-d435i-color.png" in names
    assert "s001-d435i-color-2.png" in names
    assert len(names) == 6  # 两组 × (color, depth, meta)
