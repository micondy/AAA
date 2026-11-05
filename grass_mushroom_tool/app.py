#!/usr/bin/env python3
"""草菇采集台 —— 图形化现场采集服务。

独立运行、只依赖 Python 标准库（Pillow / ROS 2 为可选增强）：
所有照片按“样本编号 + 阶段 + 部位”自动命名，不会出现 xxxx.jpg。

    python3 app.py                # 默认 http://0.0.0.0:8770
    ./start.sh                    # 自动载入 ROS 2 环境（若存在）
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import re
import shutil
import socket
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

from gmkit import media, naming
from gmkit import store as store_mod
from gmkit.rosbridge import BagRecorder, RosBridge, RosError
from gmkit.video import ExtractionJob, video_topics
from gmkit.playback import PreviewJob

WEB_DIR = Path(__file__).resolve().parent / "web"
MAX_UPLOAD = 64 * 1024 * 1024
EXPORTS = {"measurements.csv", "photos.csv", "environment.csv"}
STATIC_FILES = {
    "index.html",
    "app.js",
    "style.css",
    "manifest.webmanifest",
    "sw.js",
    "icon-192.png",
    "icon-512.png",
}
STREAM_MODES = {"color.mjpg": "color", "depth.mjpg": "depth", "rgbd.mjpg": "rgbd"}


class HttpError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status
        self.message = message


class App:
    """把存储、录包、相机快照拼在一起，供 HTTP 层调用。"""

    def __init__(self, data_root, bag_root, camera_prefix, enable_ros=True):
        self.store = store_mod.Store(data_root)
        self.bag = BagRecorder(bag_root)
        self.video = BagRecorder(Path(bag_root) / "rgbd_video")
        self.extract_job = ExtractionJob()
        self.preview_job = PreviewJob()
        self._capture_lock = threading.Lock()
        self.camera_prefix = camera_prefix
        self.ros = RosBridge(camera_prefix) if enable_ros else None
        self.urls = []
        self.addresses = []
        # 多路取景共享同一帧编码结果：N 个观众只编码一次
        self._preview_lock = threading.Lock()
        self._preview_cache = {}
        self._stream_lock = threading.Lock()
        self.streams = 0
        if self.ros is not None and self.ros.available():
            self.ros.start()

    # -- 实时画面 ---------------------------------------------------------

    def preview_frame(self, mode: str, width: int, quality: int):
        if self.ros is None:
            return None, -1
        key = (mode, width, quality)
        with self._preview_lock:
            cached = self._preview_cache.get(key)
            if cached is not None and cached[0] == self.ros.frame_seq() and cached[1]:
                return cached[1], cached[0]
        payload, seq = self.ros.preview(mode=mode, width=width, quality=quality)
        with self._preview_lock:
            self._preview_cache[key] = (seq, payload)
        return payload, seq

    def stream_open(self) -> None:
        with self._stream_lock:
            self.streams += 1

    def stream_close(self) -> None:
        with self._stream_lock:
            self.streams = max(0, self.streams - 1)

    # -- 状态 -------------------------------------------------------------

    def state(self) -> dict:
        session = self.store.current
        if self.ros is not None:
            camera = self.ros.status()
        else:
            camera = {
                "available": False,
                "reason": "已通过 --no-ros 禁用",
                "running": False,
                "prefix": self.camera_prefix,
                "age": {},
            }
        return {
            "session": session.summary() if session else None,
            "sessions": self.store.list_sessions(),
            "samples": session.samples if session else [],
            "environment": session.shots if session else [],
            "coverage": session.coverage() if session else None,
            "slots": list(naming.SLOTS),
            "env_views": list(naming.ENV_VIEWS),
            "required_slots": list(naming.REQUIRED_SLOTS),
            "d435i_slots": list(naming.D435I_SLOTS),
            "camera": camera,
            "bag": self.bag.status(),
            "video": self.video.status(),
            "video_recordings": self.video_recordings(),
            "extraction": self.extract_job.status(),
            "video_preview": self.preview_job.status(),
            "disk": _disk_usage(self.store.root),
            "streams": self.streams,
            "data_root": str(self.store.root),
            "bag_root": str(self.bag.root),
            "urls": self.urls,
            "addresses": self.addresses,
            "server_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "pillow": media.has_pillow(),
        }

    def video_recordings(self):
        recordings = []
        active = self.video.status()
        for sidecar in sorted(self.video.root.glob("*.recording.json"), reverse=True):
            try:
                entry = json.loads(sidecar.read_text(encoding="utf-8"))
                name = entry["name"]
                entry["ready"] = (self.video.root / name / "metadata.yaml").is_file() and not (active["running"] and active["name"] == name)
                preview = self.video.root / "previews" / f"{name}-rgbd.mp4"
                entry["preview_ready"] = preview.is_file()
                recordings.append(entry)
            except (OSError, ValueError, KeyError):
                continue
        return recordings

    def shutdown(self) -> None:
        try:
            status = self.bag.status()
            if status.get("running"):
                self.bag.stop()
        except Exception:
            pass
        if self.video.status().get("running"):
            self.video.stop()
        if self.ros is not None:
            self.ros.shutdown()


class Handler(BaseHTTPRequestHandler):
    server_version = "GrassMushroomTool/1.0"
    protocol_version = "HTTP/1.1"

    # -- 基础设施 ---------------------------------------------------------

    @property
    def app(self) -> App:
        return self.server.app

    def log_message(self, fmt, *args):  # 静默：现场不需要每张照片都刷日志
        pass

    def handle(self):
        try:
            super().handle()
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError):
            pass

    def _parsed(self):
        parts = urlsplit(self.path)
        return parts.path, parse_qs(parts.query)

    def _send(self, status, body: bytes, content_type: str, extra=None):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if self.close_connection:
            self.send_header("Connection", "close")
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False, default=str).encode("utf-8")
        self._send(status, body, "application/json; charset=utf-8")

    def _read_body(self, limit=MAX_UPLOAD) -> bytes:
        cached = getattr(self, "_body_cache", None)
        if cached is not None:
            if len(cached) > limit:
                raise HttpError(413, "请求体超过当前接口大小限制")
            return cached
        if self.headers.get("Transfer-Encoding"):
            self.close_connection = True
            raise HttpError(400, "暂不支持分块请求，请使用 Content-Length")
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            self.close_connection = True
            raise HttpError(400, "Content-Length 不合法")
        if length < 0 or length > limit:
            self.close_connection = True
            raise HttpError(413, "文件太大（上限 64 MB）")
        self._body_cache = self.rfile.read(length) if length else b""
        return self._body_cache

    def _json_body(self) -> dict:
        raw = self._read_body(2 * 1024 * 1024)
        if not raw:
            return {}
        try:
            data = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise HttpError(400, f"请求体不是合法 JSON: {exc}") from exc
        return data if isinstance(data, dict) else {}

    def _guard(self, func):
        # HTTP/1.1 同一 Handler 会处理多个请求。每次先读完并缓存请求体，
        # 包括不需要 JSON 的 stop/DELETE/未知路由，防止残留字节变成下一条请求的方法名。
        self._body_cache = None
        try:
            self._read_body()
            func()
        except HttpError as exc:
            self._json({"error": exc.message}, exc.status)
        except (store_mod.StoreError, RosError, naming.NamingError) as exc:
            self._json({"error": str(exc)}, 400)
        except BrokenPipeError:
            pass
        except Exception as exc:  # pragma: no cover - 兜底
            self._json({"error": f"{type(exc).__name__}: {exc}"}, 500)

    # -- 路由 -------------------------------------------------------------

    def do_GET(self):
        self._guard(self._get)

    def do_HEAD(self):
        self._guard(self._get)

    def do_POST(self):
        self._guard(self._post)

    def do_PATCH(self):
        self._guard(self._patch)

    def do_DELETE(self):
        self._guard(self._delete)

    def _get(self):
        path, query = self._parsed()

        if path in ("/", "/index.html"):
            return self._static("index.html")
        if path == "/favicon.ico":
            return self._send(204, b"", "image/x-icon")
        if path.lstrip("/") in STATIC_FILES:
            return self._static(path.lstrip("/"))
        if path == "/video-preview":
            return self._video_preview(query)
        if path == "/api/state":
            return self._json(self.app.state())
        if path.startswith("/stream/"):
            return self._stream(path[len("/stream/"):], query)
        if path == "/api/ping":
            return self._json({"ok": True, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "client": self.client_address[0]})
        if path.startswith("/media/"):
            return self._media(unquote(path[len("/media/"):]))
        if path.startswith("/thumb/"):
            raw = query.get("w", ["420"])[0]
            width = max(80, min(1200, int(raw) if str(raw).isdigit() else 420))
            return self._media(unquote(path[len("/thumb/"):]), width=width)
        if path.startswith("/export/"):
            return self._export(unquote(path[len("/export/"):]))
        raise HttpError(404, f"未知路径: {path}")

    def _post(self):
        path, query = self._parsed()

        if path == "/api/session":
            body = self._json_body()
            session = self.app.store.create(
                date=body.get("date"),
                operator=body.get("operator", ""),
                rooms=body.get("rooms"),
                notes=body.get("notes", ""),
                suffix=body.get("suffix", ""),
            )
            return self._json({"ok": True, "session": session.summary()})
        if path == "/api/session/open":
            body = self._json_body()
            session = self.app.store.open(body.get("id", ""))
            return self._json({"ok": True, "session": session.summary()})
        if path == "/api/session/config":
            body = self._json_body()
            return self._json({"ok": True, "session": self.app.store.update_config(body)})
        if path == "/api/sample":
            body = self._json_body()
            sample = self.app.store.add_sample(
                room=body.get("room"),
                row=body.get("row"),
                layer=body.get("layer"),
                mode=body.get("mode", "individual"),
                cluster_count=body.get("cluster_count"),
            )
            return self._json({"ok": True, "id": sample["id"], "sample": sample})
        if path == "/api/photo":
            return self._upload(query)
        if path == "/api/photo/delete":
            body = self._json_body()
            return self._json({"ok": True, **self.app.store.delete_photo(body.get("rel", ""))})
        if path == "/api/bag/start":
            body = self._json_body()
            return self._json({"ok": True, "bag": self.app.bag.start(body.get("name"))})
        if path == "/api/bag/stop":
            return self._json({"ok": True, "bag": self.app.bag.stop()})
        if path == "/api/video/start":
            body = self._json_body()
            if self.app.ros is None or not self.app.ros.available():
                raise HttpError(400, "未启用 ROS 相机，不能录制 RGB-D")
            camera = self.app.ros.status()
            if not camera.get("aligned") or camera["age"].get("aligned_depth", 999) is None or camera["age"].get("aligned_depth", 999) > 5:
                raise HttpError(400, "请先启动相机 stage:=full，等待对齐深度就绪")
            session = self.app.store.require()
            sid = naming.normalize_sample_id(body.get("sample"))
            session.get_sample(sid)
            context = {"session": session.id, "sample": sid, "camera_prefix": self.app.camera_prefix,
                       "format": "lossless_rgbd_rosbag2", "pointcloud": "derived_from_synchronized_rgbd_on_extraction"}
            return self._json({"ok": True, "video": self.app.video.start(
                f"{session.id}-{sid}-{time.strftime('%H%M%S')}",
                topics=video_topics(self.app.camera_prefix), context=context)})
        if path == "/api/video/stop":
            return self._json({"ok": True, "video": self.app.video.stop()})
        if path == "/api/video/preview":
            body = self._json_body()
            name = str(body.get("name", ""))
            record = next((item for item in self.app.video_recordings() if item["name"] == name and item["ready"]), None)
            if record is None:
                raise HttpError(400, "请选择已正常停止的录制")
            output = self.app.video.root / "previews" / f"{name}-rgbd.mp4"
            if output.is_file():
                return self._json({"ok": True, "ready": True})
            return self._json({"ok": True, "preview": self.app.preview_job.start(
                self.app.video.root / name, output, record["camera_prefix"])})
        if path == "/api/video/extract":
            body = self._json_body()
            name = str(body.get("name", ""))
            if not name or Path(name).name != name or name in (".", ".."):
                raise HttpError(400, "录制名称不合法")
            records = {record["name"]: record for record in self.app.video_recordings()}
            record = records.get(name)
            if record is None or not record["ready"]:
                raise HttpError(400, "请选择已停止且正常收尾的 RGB-D 录制")
            output = self.app.store.root / f"rgbd_extract_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns() % 1000000:06d}"
            return self._json({"ok": True, "extraction": self.app.extract_job.start(
                self.app.video.root / name, output, prefix=record["camera_prefix"],
                sample=record["sample"], interval=float(body.get("interval", 0.5)))})
        if path == "/api/snapshot":
            body = self._json_body()
            if self.app.ros is None:
                raise HttpError(400, "ROS 功能已禁用")
            with self.app._capture_lock:
                session = self.app.store.require()
                sid = naming.normalize_sample_id(body.get("sample"))
                session.get_sample(sid)
                seq = naming.next_d435i_seq(session.root, sid)
                files = self.app.ros.snapshot(session.root, sid,
                    stride=int(body.get("stride") or 2), want_points=bool(body.get("points", True)), seq=seq)
                entries = session.add_sensor_files(sid, files)
            return self._json({"ok": True, "group": seq, "files": entries})
        raise HttpError(404, f"未知路径: {path}")

    def _patch(self):
        path, _query = self._parsed()
        if path.startswith("/api/sample/"):
            sid = unquote(path[len("/api/sample/"):])
            body = self._json_body()
            sample = self.app.store.update_sample(sid, body)
            return self._json({"ok": True, "sample": sample})
        raise HttpError(404, f"未知路径: {path}")

    def _delete(self):
        path, _query = self._parsed()
        if path.startswith("/api/sample/"):
            sid = unquote(path[len("/api/sample/"):])
            removed = self.app.store.delete_sample(sid)
            return self._json({"ok": True, "removed": removed})
        raise HttpError(404, f"未知路径: {path}")

    # -- 具体处理 ---------------------------------------------------------

    def _video_preview(self, query):
        name = _one(query, "name", "")
        if not any(item["name"] == name for item in self.app.video_recordings()):
            raise HttpError(404, "录制不存在")
        path = self.app.video.root / "previews" / f"{name}-rgbd.mp4"
        if not path.is_file():
            raise HttpError(404, "请先生成播放预览")
        size = path.stat().st_size
        start, end, status = 0, size - 1, 200
        header = self.headers.get("Range")
        if header:
            match = re.fullmatch(r"bytes=(\d*)-(\d*)", header)
            if not match or not any(match.groups()):
                return self._send(416, b"", "video/mp4", {"Content-Range": f"bytes */{size}"})
            left, right = match.groups()
            if left:
                start = int(left)
                end = min(int(right), size - 1) if right else size - 1
            else:
                start = max(0, size-int(right))
            if start > end or start >= size:
                return self._send(416, b"", "video/mp4", {"Content-Range": f"bytes */{size}"})
            status = 206
        self.send_response(status)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(end-start+1))
        if status == 206:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        if self.command == "HEAD":
            return
        with path.open("rb") as stream:
            stream.seek(start)
            remaining = end-start+1
            while remaining:
                chunk = stream.read(min(1024*1024, remaining))
                if not chunk:
                    break
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def _upload(self, query):
        target = _one(query, "target", "sample")
        source = _one(query, "source", "phone")
        data = self._read_body()
        if not data:
            raise HttpError(400, "上传内容为空")
        ext = media.guess_ext(data)
        if target == "sample":
            entry = self.app.store.add_sample_photo(
                _one(query, "sample", ""), _one(query, "slot", ""), data, ext, source
            )
        elif target == "env":
            entry = self.app.store.add_env_shot(
                _one(query, "room", ""),
                _one(query, "view", ""),
                data,
                row=_one(query, "row", None),
                layer=_one(query, "layer", None),
                ext=ext,
                source=source,
            )
        else:
            raise HttpError(400, f"未知的上传目标: {target}")
        self._json({"ok": True, **entry})

    def _stream(self, name, query):
        """MJPEG 实时画面：手机浏览器里就是一个取景器。

        可用查询参数调节带宽：``?w=480&fps=6&q=60``（Jetson 或弱 Wi-Fi 时降低）。
        """
        mode = STREAM_MODES.get(name)
        if not mode:
            raise HttpError(404, f"未知视频流: {name}")
        ros = self.app.ros
        if ros is None or not ros.available():
            raise HttpError(503, "实时画面需要 ROS 环境与 D435i")
        if not media.has_pillow():
            raise HttpError(503, "实时画面需要安装 Pillow")

        width = _int_query(query, "w", 640, 240, 1280)
        fps = _int_query(query, "fps", 10, 2, 20)
        quality = _int_query(query, "q", 70, 40, 90)

        placeholder = media.solid_jpeg(320, 180)
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True

        last_seq = -1
        last_send = 0.0
        self.app.stream_open()
        try:
            while True:
                payload, seq = self.app.preview_frame(mode, width, quality)
                now = time.monotonic()
                if payload is None:
                    if now - last_send > 1.0:  # 没有画面时发心跳，避免 <img> 变成破图
                        payload = placeholder
                        seq = last_seq  # 不当作新画面
                    else:
                        time.sleep(1.0 / fps)
                        continue
                elif seq == last_seq:
                    time.sleep(1.0 / fps)
                    continue

                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(payload)
                )
                self.wfile.write(payload)
                self.wfile.write(b"\r\n")
                last_seq = seq
                last_send = now
                time.sleep(1.0 / fps)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return
        finally:
            self.app.stream_close()

    def _static(self, name):
        path = (WEB_DIR / name).resolve()
        try:
            path.relative_to(WEB_DIR.resolve())
        except ValueError:
            raise HttpError(404, name)
        if not path.is_file():
            raise HttpError(404, f"缺少前端文件: {name}")
        ctype = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype in ("application/javascript", "application/manifest+json"):
            ctype += "; charset=utf-8"
        self._send(200, path.read_bytes(), ctype)

    def _media(self, rel, width=None):
        path = self.app.store.resolve_media(rel)
        if not path.is_file():
            raise HttpError(404, f"文件不存在: {rel}")
        # 16 位深度 PNG 浏览器直接显示会发黑，统一按伪彩出图（原始的仍在磁盘上）
        is_depth = rel.endswith("d435i-depth.png")
        if is_depth or width:
            target_width = width or 1024
            session = self.app.store.require()
            cache = (session.root / ".thumbs" / str(target_width) / rel).with_suffix(".thumb.jpg")
            if not cache.is_file() or cache.stat().st_mtime < path.stat().st_mtime:
                media.make_thumbnail(path, cache, target_width)
            if cache.is_file():
                return self._send(
                    200,
                    cache.read_bytes(),
                    "image/jpeg",
                    {"Cache-Control": "public, max-age=86400"},
                )
        ctype = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        self._send(200, path.read_bytes(), ctype, {"Cache-Control": "public, max-age=3600"})

    def _export(self, name):
        if name not in EXPORTS:
            raise HttpError(404, f"不支持导出: {name}")
        session = self.app.store.require()
        path = session.root / name
        if not path.is_file():
            raise HttpError(404, f"{name} 还没有生成")
        self._send(
            200,
            path.read_bytes(),
            "text/csv; charset=utf-8",
            {"Content-Disposition": f'attachment; filename="{session.id}-{name}"'},
        )


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, app):
        super().__init__(address, handler)
        self.app = app

    def handle_error(self, request, client_address):
        """手机切走/锁屏会直接断连，这类异常不需要刷屏。"""
        exc = sys.exc_info()[1]
        if isinstance(exc, (ConnectionResetError, BrokenPipeError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------


def _one(query, key, default=None):
    values = query.get(key)
    return values[-1] if values else default


def _int_query(query, key, default, low, high):
    raw = _one(query, key)
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return default
    return max(low, min(high, value))


def _disk_usage(path: Path) -> dict:
    try:
        usage = shutil.disk_usage(str(path))
    except OSError:
        return {}
    return {
        "total": usage.total,
        "free": usage.free,
        "used_percent": round(100.0 * usage.used / usage.total, 1) if usage.total else 0,
    }


#: 这些网卡不是手机能访问的局域网（VPN/TUN、容器、回环等）
_EXCLUDED_IFACE_PREFIXES = (
    "lo", "docker", "br-", "veth", "virbr", "vnet", "tun", "tap", "wg",
    "utun", "tailscale", "zt", "mihomo", "clash", "meta", "ppp",
)
_EXCLUDED_NETS = ("127.", "169.254.", "198.18.", "198.19.")


def _local_ip() -> str:
    """兜底：按默认路由取一个本机地址（可能拿到 VPN/TUN 地址）。"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except OSError:
        return "127.0.0.1"


def _subnet_rank(ip: str) -> int:
    if ip.startswith("192.168."):
        return 0
    if ip.startswith("10."):
        return 1
    if re.match(r"172\.(1[6-9]|2\d|3[01])\.", ip):
        return 2
    if re.match(r"100\.(6[4-9]|[7-9]\d|1[01]\d|12[0-7])\.", ip):
        return 3
    return 4


def lan_addresses() -> list:
    """列出手机可能访问到的本机局域网地址。

    代理软件的 TUN 网卡（Clash/Mihomo 的 198.18.0.1 之类）会造成
    “默认路由地址”错误，必须按网卡枚举并排除。
    """
    entries = []
    try:
        output = subprocess.run(
            ["ip", "-4", "-o", "addr", "show"],
            capture_output=True,
            text=True,
            timeout=3,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        output = ""

    for line in output.splitlines():
        match = re.match(r"^\d+:\s+(\S+)\s+inet\s+([0-9.]+)/(\d+)", line)
        if not match:
            continue
        iface, ip, prefix = match.group(1), match.group(2), int(match.group(3))
        name = iface.lower()
        if any(name.startswith(prefix_) for prefix_ in _EXCLUDED_IFACE_PREFIXES):
            continue
        if any(ip.startswith(net) for net in _EXCLUDED_NETS):
            continue
        entries.append({"iface": iface, "ip": ip, "prefix": prefix})

    if not entries:
        entries = [{"iface": "default", "ip": _local_ip(), "prefix": 0}]

    entries.sort(key=lambda item: (_subnet_rank(item["ip"]), item["iface"]))
    return entries


def _iface_kind(name: str) -> str:
    name = name.lower()
    if name.startswith("wl"):
        return "无线"
    if name.startswith(("en", "eth", "usb")):
        return "有线/USB"
    return "其它"


def _host_is_loopback(host: str) -> bool:
    return str(host).strip().lower() in {"127.0.0.1", "localhost", "::1"}


def _ufw_active() -> bool:
    """ufw 服务常驻但可能未启用，必须看 ufw.conf 的 ENABLED。"""
    try:
        for line in Path("/etc/ufw/ufw.conf").read_text(encoding="utf-8", errors="ignore").splitlines():
            if line.strip().upper().startswith("ENABLED="):
                return line.split("=", 1)[1].strip().strip('"').lower() in ("yes", "true", "1")
    except OSError:
        pass
    return False


def _firewalld_active() -> bool:
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", "firewalld"], timeout=2).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _firewall_hint(port: int):
    if _ufw_active():
        return f"sudo ufw allow {port}/tcp"
    if _firewalld_active():
        return f"sudo firewall-cmd --add-port={port}/tcp"
    return None


def main(argv=None) -> int:
    base = Path(__file__).resolve().parent
    try:  # 输出重定向到日志时也能及时看到启动信息
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    parser = argparse.ArgumentParser(description="草菇采集台：图形化现场采集界面")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址（手机访问请用 0.0.0.0）")
    parser.add_argument("--port", type=int, default=8770, help="监听端口")
    parser.add_argument("--data", default=str(base / "data"), help="采集数据根目录")
    parser.add_argument("--bags", default=str(base / "bags"), help="rosbag 保存目录")
    parser.add_argument(
        "--camera-prefix",
        default=os.environ.get("GM_CAMERA_PREFIX", "/sensors/nav_camera"),
        help="D435i 话题前缀",
    )
    parser.add_argument("--no-ros", action="store_true", help="禁用 rosbag 与 D435i 快照")
    parser.add_argument("--open", action="store_true", help="启动后自动打开浏览器")
    args = parser.parse_args(argv)

    app = App(args.data, args.bags, args.camera_prefix, enable_ros=not args.no_ros)
    try:
        httpd = Server((args.host, args.port), Handler, app)
    except OSError as exc:
        print(f"端口 {args.port} 不可用: {exc}", file=sys.stderr)
        return 1
    port = httpd.server_address[1]

    local = f"http://127.0.0.1:{port}"
    addresses = [
        {
            "ip": item["ip"],
            "iface": item["iface"],
            "prefix": item["prefix"],
            "kind": _iface_kind(item["iface"]),
            "url": f"http://{item['ip']}:{port}",
        }
        for item in lan_addresses()
    ]
    app.addresses = addresses
    app.urls = [local] + [item["url"] for item in addresses]

    def _stop(_signum=None, _frame=None):
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print("草菇采集台已启动")
    print(f"  本机访问: {local}")
    if addresses:
        print("  手机访问（与电脑同一 Wi-Fi / 同一热点下任选其一）:")
        for item in addresses:
            print(f"    {item['url']}   [{item['iface']} · {item['kind']}]")
    else:
        print("  手机访问: 未检测到局域网地址（先连上 Wi-Fi 或手机热点）")
    print(f"  数据目录: {app.store.root}")
    print(f"  录包目录: {app.bag.root}")
    camera = app.state()["camera"]
    print(
        "  D435i  : " + ("已连接订阅" if camera.get("available") else f"不可用（{camera.get('reason') or '未载入 ROS 环境'}）")
    )
    if _host_is_loopback(args.host):
        print("  提示: 当前只监听了本机地址，手机无法访问；去掉 --host 参数即可")
    firewall = _firewall_hint(port)
    if firewall:
        print(f"  提示: 检测到防火墙已启用，手机打不开时执行 {firewall}")
    print("  按 Ctrl+C 停止")

    if args.open:
        threading.Timer(0.6, lambda: webbrowser.open(local)).start()

    try:
        httpd.serve_forever(poll_interval=0.3)
    finally:
        app.shutdown()
        httpd.server_close()
        print("\n已退出。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
