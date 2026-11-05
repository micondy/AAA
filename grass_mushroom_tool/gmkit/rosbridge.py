"""可选能力：rosbag 录制 + D435i 快照。

本模块不引用任何本机 ROS 工作区的 Python 包，只使用 ``ros2`` 命令和 ``rclpy``
公共接口；没有 ROS 环境时会自动降级为“不可用”，拍照功能照常工作。
"""

from __future__ import annotations

import collections
import tempfile
import json
import os
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

try:  # pragma: no cover - 运行环境决定
    import numpy as np

    _NUMPY = True
except Exception:  # pragma: no cover
    np = None
    _NUMPY = False

from . import media, naming

BAG_COMMAND = ("ros2", "bag", "record", "-a", "--include-hidden-topics")


class RosError(RuntimeError):
    """ROS 相关操作失败，消息可直接展示给操作者。"""


# ---------------------------------------------------------------------------
# rosbag 录制
# ---------------------------------------------------------------------------


class BagRecorder:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._proc = None
        self._target = None
        self._log_path = None
        self._log_handle = None
        self._started = None
        self._last = None
        self._context = None

    # -- 状态 -------------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            if self._proc is not None and not running and self._last is None:
                self._last = {"name": self._target.name if self._target else "", "code": self._proc.returncode}
            size = _dir_size(self._target) if self._target else 0
            return {
                "available": shutil.which("ros2") is not None,
                "running": running,
                "name": self._target.name if self._target else None,
                "dir": str(self._target) if self._target else None,
                "elapsed": round(time.time() - self._started, 1) if running and self._started else 0,
                "bytes": size,
                "log": str(self._log_path) if self._log_path else None,
                "last": self._last,
                "root": str(self.root),
                "context": self._context,
            }

    # -- 控制 -------------------------------------------------------------

    def start(self, name=None, *, topics=None, context=None) -> dict:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                raise RosError("已经在录制中了")
            if shutil.which("ros2") is None:
                raise RosError("当前环境找不到 ros2 命令，请用 start.sh 启动以载入 ROS 环境")

            stamp = time.strftime("%Y%m%d-%H%M%S")
            base = _safe_name(name) or f"grass_mushroom_{time.strftime('%Y%m%d')}"
            target = self.root / base
            counter = 2
            while target.exists():
                target = self.root / f"{base}-{counter}"
                counter += 1

            self._log_path = self.root / f"{target.name}.log"
            self._log_handle = self._log_path.open("ab")
            self._log_handle.write(f"\n=== {stamp} 开始录制 -> {target} ===\n".encode("utf-8"))
            self._log_handle.flush()
            command = (list(BAG_COMMAND) if topics is None else
                       ["ros2", "bag", "record", "--disable-keyboard-controls", "-s", "mcap", "--topics", *topics])
            self._context = dict(context or {})
            self._context.update({"name": target.name, "created_at": stamp, "topics": topics, "stopped": False})
            (self.root / f"{target.name}.recording.json").write_text(
                json.dumps(self._context, ensure_ascii=False, indent=2), encoding="utf-8")
            self._proc = subprocess.Popen(
                command + ["-o", str(target)],
                stdout=self._log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                cwd=str(self.root),
            )
            self._target = target
            self._started = time.time()
            self._last = None
            return self.status()

    def stop(self) -> dict:
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                raise RosError("当前没有在录制")
            # ros2 bag record 需要 SIGINT 才能正常收尾（关闭数据库、写 metadata.yaml）
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
                proc.wait(timeout=5)
            if self._log_handle is not None:
                self._log_handle.write(f"=== 结束，返回码 {proc.returncode} ===\n".encode("utf-8"))
                self._log_handle.flush()
                self._log_handle.close()
                self._log_handle = None
            self._last = {"name": self._target.name if self._target else "", "code": proc.returncode}
            if self._context is not None:
                self._context.update({"stopped": True, "exit_code": proc.returncode,
                                      "metadata_written": (self._target / "metadata.yaml").is_file()})
                (self.root / f"{self._target.name}.recording.json").write_text(
                    json.dumps(self._context, ensure_ascii=False, indent=2), encoding="utf-8")
            return self.status()


# ---------------------------------------------------------------------------
# D435i 快照
# ---------------------------------------------------------------------------


class RosBridge:
    """订阅 D435i 话题并保持最新一帧，用于“抓取三维快照”。"""

    def __init__(self, camera_prefix="/sensors/nav_camera"):
        prefix = (camera_prefix or "").rstrip("/")
        self.camera_prefix = prefix
        # 彩色（848x480@30）与它的内参
        self.color_topic = f"{prefix}/color/image_raw"
        self.color_info_topic = f"{prefix}/color/camera_info"
        # 对齐深度：stage:=full（align_depth.enable=true）时存在，逐像素对齐彩色
        self.aligned_depth_topic = f"{prefix}/aligned_depth_to_color/image_raw"
        self.aligned_depth_info_topic = f"{prefix}/aligned_depth_to_color/camera_info"
        # 原始深度仅用于预览/诊断，快照禁止回退。
        self.raw_depth_topic = f"{prefix}/depth/image_rect_raw"
        self.raw_depth_info_topic = f"{prefix}/depth/camera_info"
        # 驱动点云仅作状态诊断；保存时从同步 RGB-D 重建彩色光学系点云
        self.cloud_topic = f"{prefix}/depth/color/points"

        # 兼容旧字段名
        self.depth_topic = self.aligned_depth_topic
        self.info_topic = self.color_info_topic

        self._lock = threading.RLock()
        self._latest = {}
        self._color_buffer = collections.deque(maxlen=16)
        self._depth_buffer = collections.deque(maxlen=16)  # 仅缓存对齐深度，避免被原始帧挤掉
        self._seq = 0  # 每收到一帧 +1，供 MJPEG 推流判断是否有新画面
        self._node = None
        self._thread = None
        self._stop = threading.Event()
        self._error = None
        self._reason = None
        self._avail = None

    # -- 能力探测 ---------------------------------------------------------

    def available(self) -> bool:
        if self._avail is None:
            if not _NUMPY:
                self._avail = False
                self._reason = "缺少 numpy"
            else:
                try:
                    import rclpy  # noqa: F401
                    from sensor_msgs.msg import Image  # noqa: F401

                    self._avail = True
                except Exception as exc:  # pragma: no cover - 运行环境
                    self._avail = False
                    self._reason = f"{type(exc).__name__}: {exc}"
        return bool(self._avail)

    def status(self) -> dict:
        now = time.monotonic()

        def age(kind):
            with self._lock:
                entry = self._latest.get(kind)
            return None if not entry else round(now - entry[1], 1)

        with self._lock:
            color = self._latest.get("color")
            aligned = next((item for item in reversed(self._depth_buffer) if item[2]), None)
            raw = self._latest.get("raw_depth")
            cloud = self._latest.get("cloud")

        depth_entry = aligned or raw
        return {
            "available": self.available(),
            "reason": self._reason,
            "running": self._node is not None,
            "error": self._error,
            "prefix": self.camera_prefix,
            "topics": {
                "color": self.color_topic,
                "depth": self.aligned_depth_topic if aligned else self.raw_depth_topic,
                "aligned_depth": self.aligned_depth_topic,
                "points": self.cloud_topic,
            },
            "aligned": aligned is not None,
            "depth_source": "aligned" if aligned else ("raw" if raw else None),
            "age": {
                "color": age("color"),
                "aligned_depth": round(now - aligned[1], 1) if aligned else None,
                "raw_depth": round(now - raw[1], 1) if raw else None,
                "cloud": age("cloud"),
                "color_info": age("color_info"),
            },
            "size": {
                "color": [color[0].width, color[0].height] if color else None,
                "depth": [depth_entry[0].width, depth_entry[0].height] if depth_entry else None,
                "cloud": int(cloud[0].width * cloud[0].height) if cloud else None,
            },
        }

    # -- 生命周期 ---------------------------------------------------------

    def start(self) -> bool:
        if not self.available():
            return False
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return True
            self._stop.clear()
            self._thread = threading.Thread(target=self._spin, name="d435i-snapshot", daemon=True)
            self._thread.start()
        return True

    def shutdown(self, timeout: float = 3.0) -> None:
        """停订阅线程并等它退出，避免解释器退出时 rclpy 在别的线程里析构而 abort。"""
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)

    def _spin(self) -> None:  # pragma: no cover - 需要真实 ROS 环境
        node = None
        try:
            import rclpy
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CameraInfo, Image, PointCloud2

            rclpy.init(args=None)
            node = rclpy.create_node("grass_mushroom_snapshot")
            qos = QoSProfile(
                depth=16,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            node.create_subscription(Image, self.color_topic, lambda m: self._keep("color", m), qos)
            node.create_subscription(CameraInfo, self.color_info_topic, lambda m: self._keep("color_info", m), qos)
            node.create_subscription(
                Image, self.aligned_depth_topic, lambda m: self._keep_depth(m, True), qos
            )
            node.create_subscription(
                Image, self.raw_depth_topic, lambda m: self._keep_depth(m, False), qos
            )
            node.create_subscription(
                CameraInfo, self.aligned_depth_info_topic, lambda m: self._keep("aligned_depth_info", m), qos
            )
            node.create_subscription(
                CameraInfo, self.raw_depth_info_topic, lambda m: self._keep("raw_depth_info", m), qos
            )
            node.create_subscription(PointCloud2, self.cloud_topic, lambda m: self._keep("cloud", m), qos)
            self._node = node
            while not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
        finally:
            self._node = None
            try:
                if node is not None:
                    node.destroy_node()
            except Exception:
                pass
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    def _keep(self, kind: str, msg) -> None:
        with self._lock:
            self._latest[kind] = (msg, time.monotonic())
            if kind == "color":
                self._color_buffer.append(self._latest[kind])
            self._seq += 1

    def _keep_depth(self, msg, aligned: bool) -> None:
        with self._lock:
            received = time.monotonic()
            if aligned:
                self._depth_buffer.append((msg, received, True))
            else:
                self._latest["raw_depth"] = (msg, received, False)
            self._seq += 1

    def frame_seq(self) -> int:
        with self._lock:
            return self._seq

    # -- 实时画面（手机取景器） -------------------------------------------

    def preview(self, mode: str = "rgbd", width: int = 640, quality: int = 70, depth_range=(150, 3000)):
        """把最新一帧转成 JPEG，供 MJPEG 推流。返回 (bytes, seq)，无画面时 bytes 为 None。"""
        with self._lock:
            color_entry = self._latest.get("color")
            seq = self._seq
            aligned = next((item for item in reversed(self._depth_buffer) if item[2]), None)
            raw = self._latest.get("raw_depth")
        if color_entry is None:
            return None, seq
        if time.monotonic() - color_entry[1] > 30:
            return None, seq

        rgb = _image_array(color_entry[0])
        if mode == "color":
            return media.to_jpeg(rgb, quality, width), seq

        depth_entry = aligned or raw
        if depth_entry is None:
            return None, seq
        depth = _image_array(depth_entry[0])
        low, high = _auto_depth_range(depth)
        depth_rgb = _colorize_depth(depth, low, high)
        if mode == "depth":
            return media.to_jpeg(depth_rgb, quality, width), seq

        import numpy as np

        rows = min(rgb.shape[0], depth_rgb.shape[0])
        cols = min(rgb.shape[1], depth_rgb.shape[1])
        side_by_side = np.concatenate([rgb[:rows, :cols], depth_rgb[:rows, :cols]], axis=1)
        return media.to_jpeg(side_by_side, quality, width), seq

    def _pick_depth(self, color_msg, tolerance: float = 0.0):
        """只接受相机 header 时间戳；默认必须完全相同，不按到达时间凑帧。"""
        stamp = _stamp_ns(color_msg)
        if stamp is None:
            return None, None
        with self._lock:
            entries = [item for item in self._depth_buffer if item[2] and _stamp_ns(item[0]) is not None]
        if not entries:
            return None, None
        best = min(entries, key=lambda item: abs(_stamp_ns(item[0]) - stamp))
        delta_ns = abs(_stamp_ns(best[0]) - stamp)
        return (best if delta_ns <= round(tolerance * 1e9) else None), delta_ns / 1e9

    def _pair(self, timeout: float = 2.5):
        """从两路缓存中找最新的完整同步帧，允许回调先后到达。"""
        deadline = time.monotonic() + timeout
        last_dt = None
        color_entry = None
        while True:
            with self._lock:
                colors = list(self._color_buffer)
                color_entry = self._latest.get("color")
                if color_entry is not None and not any(item[0] is color_entry[0] for item in colors):
                    colors.append(color_entry)
            for entry in reversed(colors):
                picked, delta = self._pick_depth(entry[0])
                if picked is not None and time.monotonic() - entry[1] < 5 and time.monotonic() - picked[1] < 5:
                    return entry, picked, delta
                if delta is not None:
                    last_dt = delta
            if time.monotonic() >= deadline:
                return color_entry, None, last_dt
            time.sleep(0.01)

    # -- 快照 -------------------------------------------------------------

    def snapshot(self, session_root, sid, *, stride: int = 2, want_points: bool = True, seq: int = 1) -> list:
        """把一帧配对的 RGB-D 写进样本目录，返回 [{kind, rel, bytes}]。

        深度只使用 ``aligned_depth_to_color``（与彩色逐像素对齐、单位 mm）；
        缺失、过期或配对失败时拒绝保存，禁止回退到原始深度。
        ``seq`` 用于同一编号下保存多组 RGB-D（第二组起文件名加 ``-2``）。
        """
        if not self.available():
            raise RosError(f"D435i 快照不可用：{self._reason or '未启用 ROS'}")
        if not self.start():
            raise RosError("D435i 订阅线程未启动")

        session_root = Path(session_root)
        with self._lock:
            first = self._latest.get("color")
        if first is None:
            raise RosError("还没有收到彩色图像：请先启动 D435i（stage:=full）并等待几秒")
        if time.monotonic() - first[1] > 30:
            raise RosError("彩色图像超过 30 秒未更新：请检查相机是否还在运行")

        # enable_sync 下彩色与对齐深度来自同一个 frameset；这里再按时间戳确认配对
        color_entry, depth_entry, delta = self._pair()
        if depth_entry is None:
            if delta is None:
                raise RosError("未收到对齐深度，未保存快照：请确认相机以 stage:=full 启动（align_depth.enable=true）；原始深度仅供预览")
            raise RosError(
                "对齐深度已过期或与彩色时间戳对不上（Δt≈{:.0f} ms），未保存快照：请确认相机用 stage:=full（已启用对齐深度），"
                "抓取时保持相机静止后再点一次".format(delta * 1000)
            )

        with self._lock:
            color_info = self._latest.get("color_info")
            depth_info = self._latest.get("aligned_depth_info")
        if not depth_entry[2]:
            raise RosError("深度未对齐，未保存快照：禁止使用原始深度")
        return self.save_pair(session_root, sid, color_entry[0], depth_entry[0],
                              color_info[0] if color_info else None,
                              depth_info[0] if depth_info else None,
                              stride=stride, want_points=want_points, seq=seq)

    def save_pair(self, session_root, sid, color_msg, depth_msg, color_info, depth_info=None,
                  *, stride=2, want_points=True, seq=1, source=None):
        """保存已对齐、同一时间戳的图像；供现场快照及离线抽帧共用。"""
        session_root = Path(session_root)
        if _stamp_ns(color_msg) is None or _stamp_ns(color_msg) != _stamp_ns(depth_msg):
            raise RosError("RGB 与对齐深度时间戳不相同，未保存快照")
        if (color_msg.width, color_msg.height) != (depth_msg.width, depth_msg.height):
            raise RosError("彩色与对齐深度尺寸不一致，未保存快照")
        if not color_msg.header.frame_id or color_msg.header.frame_id != depth_msg.header.frame_id:
            raise RosError("彩色与对齐深度光学坐标系不一致，未保存快照")
        if depth_msg.encoding.lower() not in ("16uc1", "mono16"):
            raise RosError("深度必须为 16 位毫米数据，未保存快照")
        if int(stride) < 1:
            raise RosError("点云抽样步长必须大于 0")
        if want_points:
            _validate_color_info(color_info, color_msg)
        kinds = ["color", "depth"] + (["points"] if want_points else []) + ["meta"]
        targets = {kind: session_root / naming.d435i_rel(sid, kind, seq=seq) for kind in kinds}
        if any(path.exists() for path in targets.values()):
            raise RosError(f"第 {seq} 组文件已存在，拒绝覆盖")
        session_root.mkdir(parents=True, exist_ok=True)
        rgb, depth = _image_array(color_msg), _image_array(depth_msg)
        written = []
        # 先完成编码与点云计算，再提交文件；meta 最后写入，作为完整组的标记。
        with tempfile.TemporaryDirectory(prefix=".rgbd-", dir=session_root) as stage:
            stage = Path(stage)
            media.write_png(stage / "color", rgb)
            media.write_png(stage / "depth", depth)
            count = _write_rgbd_ply(stage / "points", rgb, depth, color_info, int(stride)) if want_points else 0
            payload = _meta(color=color_msg, depth=depth_msg, aligned=True, delta=0,
                            color_info=color_info, depth_info=depth_info,
                            cloud=None, points_written=count, stride=stride, bridge=self, seq=seq)
            if want_points:
                payload["pointcloud"] = {
                    "method": "backproject_aligned_depth_with_color_intrinsics",
                    "topic": None, "frame_id": color_msg.header.frame_id,
                    "stamp": _stamp_dict(depth_msg), "color_stamp": _stamp_dict(color_msg),
                    "dt_ms": 0.0, "unit": "m", "stride": int(stride),
                    "points_written": count, "points_total": int(np.count_nonzero(depth)),
                }
            if source is not None:
                payload["source"] = source
            (stage / "meta").write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            created = []
            try:
                for kind in kinds:
                    target = targets[kind]
                    target.parent.mkdir(parents=True, exist_ok=True)
                    # 独占创建防止并发覆盖；失败时仅回滚本次创建的文件。
                    with target.open("xb") as stream:
                        created.append(target)
                        with (stage / kind).open("rb") as src:
                            shutil.copyfileobj(src, stream)
                    written.append({"kind": kind, "rel": target.relative_to(session_root).as_posix(), "bytes": target.stat().st_size})
            except Exception:
                for path in created:
                    path.unlink(missing_ok=True)
                raise
        return written


# ---------------------------------------------------------------------------
# 内部工具
# ---------------------------------------------------------------------------


def _safe_name(value) -> str:
    text = str(value or "").strip()
    cleaned = "".join(ch if (ch.isalnum() or ch in "-_") else "-" for ch in text)
    return cleaned.strip("-_")[:60]


def _dir_size(path) -> int:
    if not path or not Path(path).is_dir():
        return 0
    total = 0
    for child in Path(path).rglob("*"):
        try:
            if child.is_file():
                total += child.stat().st_size
        except OSError:
            continue
    return total


def _image_array(msg):  # pragma: no cover - 需要真实消息
    """sensor_msgs/Image -> numpy 数组（RGB 或 uint16 深度）。"""
    height, width, step = msg.height, msg.width, msg.step
    raw = msg.data
    if isinstance(raw, (bytes, bytearray, memoryview)):
        buffer = np.frombuffer(raw, dtype=np.uint8)
    else:
        buffer = np.asarray(raw, dtype=np.uint8)

    encoding = (msg.encoding or "").lower()
    if encoding in ("rgb8", "bgr8"):
        rows = buffer[: height * step].reshape(height, step)
        image = rows[:, : width * 3].reshape(height, width, 3)
        return image[:, :, ::-1] if encoding == "bgr8" else image
    if encoding in ("rgba8", "bgra8"):
        rows = buffer[: height * step].reshape(height, step)
        image = rows[:, : width * 4].reshape(height, width, 4)
        return image[:, :, [2, 1, 0]] if encoding == "bgra8" else image[:, :, :3]
    if encoding in ("mono8", "8uc1"):
        rows = buffer[: height * step].reshape(height, step)
        return rows[:, :width]
    if encoding in ("16uc1", "mono16"):
        dtype = ">u2" if getattr(msg, "is_bigendian", False) else "<u2"
        pairs = buffer[: height * step].view(dtype).reshape(height, step // 2)
        return pairs[:, :width].astype(np.uint16, copy=False)
    raise RosError(f"不支持的图像编码: {msg.encoding}")


def _validate_color_info(info, color):
    if info is None or (info.width, info.height) != (color.width, color.height):
        raise RosError("缺少匹配的彩色相机内参，无法生成同步点云，未保存快照")
    if info.header.frame_id != color.header.frame_id or len(info.k) != 9:
        raise RosError("彩色相机内参坐标系不匹配，未保存快照")
    k = np.asarray(info.k, dtype=float)
    if not np.all(np.isfinite(k)) or k[0] <= 0 or k[4] <= 0:
        raise RosError("彩色相机内参无效，未保存快照")
    if any(abs(float(v)) > 1e-12 for v in info.d):
        raise RosError("彩色图像存在非零畸变，需先校正后生成点云，未保存快照")


def _write_rgbd_ply(path, rgb, depth, info, stride=2):
    """从同一 RGB-D 帧反投影，输出彩色光学系、米单位的彩色点云。"""
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise RosError("生成彩色点云需要三通道 RGB 图像")
    y, x = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride].astype(np.float64) / 1000.0
    valid = z > 0
    x, y, z = x[valid], y[valid], z[valid]
    if not z.size:
        raise RosError("当前深度没有有效点，未保存快照")
    out = np.empty(z.size, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    out["x"] = (x - info.k[2]) * z / info.k[0]
    out["y"] = (y - info.k[5]) * z / info.k[4]
    out["z"] = z
    colors = rgb[y, x]
    out["red"], out["green"], out["blue"] = colors[:, 0], colors[:, 1], colors[:, 2]
    header = ("ply\nformat binary_little_endian 1.0\n"
              "comment synchronized RGB-D; color optical frame; meters\n"
              f"element vertex {z.size}\nproperty float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    with Path(path).open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(out.tobytes())
    return int(z.size)


def _write_ply(path, cloud, stride: int = 2) -> int:  # pragma: no cover - 需要真实消息
    from sensor_msgs_py import point_cloud2

    names = [field.name for field in cloud.fields]
    fields = ("x", "y", "z", "rgb") if "rgb" in names else ("x", "y", "z")
    points = point_cloud2.read_points(cloud, field_names=fields, skip_nans=True)
    points = np.asarray(points)
    if points.size == 0:
        raise RosError("点云为空")
    if stride and stride > 1:
        points = points[:: int(stride)]

    count = int(points.shape[0])
    out = np.empty(count, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    out["x"] = points["x"]
    out["y"] = points["y"]
    out["z"] = points["z"]
    if "rgb" in fields:
        packed = np.asarray(points["rgb"]).view(np.uint32)
        out["red"] = ((packed >> 16) & 0xFF).astype("u1")
        out["green"] = ((packed >> 8) & 0xFF).astype("u1")
        out["blue"] = (packed & 0xFF).astype("u1")
    else:
        out["red"] = out["green"] = out["blue"] = 255

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment grass_mushroom_tool d435i snapshot\n"
        f"element vertex {count}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    with Path(path).open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(out.tobytes())
    return count


_DEPTH_COLORS = None


def _depth_colormap():
    """兼容旧调用；色表实现已移到 media.depth_colormap。"""
    return media.depth_colormap()


def _auto_depth_range(depth, fallback=(150, 3000)):
    """按当前场景的有效深度自动拉伸伪彩范围，避免远处全黑看不清。"""
    valid = depth[depth > 0]
    if valid.size < max(64, depth.size * 0.05):
        return fallback
    low = float(np.percentile(valid, 2))
    high = float(np.percentile(valid, 98))
    if high - low < 200:
        high = low + 200
    return low, high


def _colorize_depth(depth, low=None, high=None):
    """深度（uint16 mm）-> RGB：近处偏红、远处偏蓝、无效值涂黑。"""
    return media.colorize_depth(depth, low, high)


def _stamp_ns(msg):
    """使用整数纳秒避免 epoch 秒转换为 float 后丢失精度。"""
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    value = int(stamp.sec * 1_000_000_000) + int(stamp.nanosec)
    return value if value > 0 else None


def _stamp_seconds(msg):
    """header.stamp -> 秒（浮点）；没有有效时间戳时返回 None。"""
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return value if value > 0 else None


def _stamp_dict(msg):
    if msg is None:
        return None
    return {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)}


def _camera_info_dict(info) -> dict:
    if info is None:
        return None
    return {
        "width": info.width,
        "height": info.height,
        "k": [round(float(v), 6) for v in info.k],
        "d": [round(float(v), 9) for v in info.d],
        "frame_id": info.header.frame_id,
        "stamp": _stamp_dict(info),
    }


def _meta(*, color, depth, aligned, delta, color_info, depth_info, cloud, points_written, stride, bridge, seq=1) -> dict:  # pragma: no cover - 需要真实消息
    """记录这帧 RGB-D 的全部关键信息，方便事后复现尺寸估计。"""
    depth_topic = bridge.aligned_depth_topic if aligned else bridge.raw_depth_topic
    return {
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "group": int(seq),
        "camera_prefix": bridge.camera_prefix,
        "alignment": {
            "aligned_to_color": bool(aligned),
            "method": "realsense align_depth.enable=true" if aligned else "raw depth (未对齐，需用深度内参加 depth_to_color 外参自行投影)",
            "dt_ms": round(float(delta or 0) * 1000, 1),
        },
        "color": {
            "topic": bridge.color_topic,
            "width": color.width,
            "height": color.height,
            "encoding": color.encoding,
            "frame_id": color.header.frame_id,
            "stamp": _stamp_dict(color),
        },
        "depth": {
            "topic": depth_topic,
            "width": depth.width,
            "height": depth.height,
            "encoding": depth.encoding,
            "frame_id": depth.header.frame_id,
            "stamp": _stamp_dict(depth),
            "unit": "mm" if depth.encoding.lower() in ("16uc1", "mono16") else "raw",
            "scale_mm": 1 if depth.encoding.lower() in ("16uc1", "mono16") else None,
            "hole_value": 0,
        },
        "color_camera_info": _camera_info_dict(color_info),
        "depth_camera_info": _camera_info_dict(depth_info),
        "pointcloud": {
            "topic": bridge.cloud_topic,
            "frame_id": cloud.header.frame_id if cloud is not None else None,
            "stride": int(stride),
            "points_written": int(points_written),
            "points_total": int(cloud.width * cloud.height) if cloud is not None else 0,
        },
    }
