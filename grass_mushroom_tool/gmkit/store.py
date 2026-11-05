"""会话数据存储：目录结构、样本索引、照片落盘、CSV 导出。

目录结构::

    data/
      current.txt                     # 当前会话 id
      grass_mushroom_20261105/
        session.json                  # 采集日期、采集员、菇房配置、编号游标
        samples.json                  # 样本索引（权威数据）
        environment.json              # 环境照片索引
        measurements.csv              # 人工测量表（由 samples.json 生成）
        photos.csv                    # 全部照片清单（由索引生成）
        environment.csv               # 环境照片清单（由索引生成）
        samples/S001/s001-inplace-far.jpg ...
        environment/A/env-A-r02-l01-bed.jpg ...
"""

from __future__ import annotations

import csv
import json
import os
import re
import threading
from datetime import datetime
from pathlib import Path

from . import naming

MODE_LABELS = {"individual": "单个", "cluster": "成簇"}


class StoreError(RuntimeError):
    """可以直接展示给现场操作者的错误。"""


def _now() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


def _load_json(path: Path, default):
    if not path.is_file():
        return json.loads(json.dumps(default))  # 深拷贝
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise StoreError(f"无法读取 {path.name}: {exc}") from exc
    return data if data is not None else json.loads(json.dumps(default))


def _opt_int(value, field: str):
    if value is None or (isinstance(value, str) and not value.strip()):
        return None
    try:
        number = int(float(str(value).strip()))
    except (TypeError, ValueError) as exc:
        raise StoreError(f"{field} 必须是整数: {value!r}") from exc
    if number < 0:
        raise StoreError(f"{field} 不能为负数: {number}")
    return number


def _opt_float(value, field: str):
    if value is None or (isinstance(value, str) and not value.strip()):
        return None
    try:
        number = float(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise StoreError(f"{field} 必须是数字: {value!r}") from exc
    if number < 0:
        raise StoreError(f"{field} 不能为负数: {number}")
    return round(number, 2)


def parse_rooms(text) -> list:
    """解析“菇房配置”文本。

    每行一间菇房：``名称[,排数[,层数]]``，例如::

        A,12,3
        B 房,10,3
        C
    """
    if isinstance(text, list):
        rooms = []
        for item in text:
            if isinstance(item, dict):
                rooms.append(
                    {
                        "name": naming.slug(item.get("name"), fallback="") or "菇房",
                        "rows": _opt_int(item.get("rows"), "排数") or 0,
                        "layers": _opt_int(item.get("layers"), "层数") or 0,
                    }
                )
        return rooms

    rooms = []
    for raw_line in str(text or "").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = [p.strip() for p in re.split(r"[,，;；\t]+", line) if p.strip()]
        if not parts:
            continue
        rows = _opt_int(parts[1], "排数") if len(parts) > 1 else 0
        layers = _opt_int(parts[2], "层数") if len(parts) > 2 else 0
        rooms.append(
            {
                "name": naming.slug(parts[0], fallback="") or "菇房",
                "rows": rows or 0,
                "layers": layers or 0,
            }
        )
    return rooms


class Session:
    """一个采集会话（通常等于一次现场采集、一个日期）。"""

    def __init__(self, root: Path):
        self.root = Path(root)
        self.config = _load_json(self.root / "session.json", {})
        self.samples = _load_json(self.root / "samples.json", {"samples": []}).get("samples", [])
        self.shots = _load_json(self.root / "environment.json", {"shots": []}).get("shots", [])

    # -- 基本信息 ---------------------------------------------------------

    @property
    def id(self) -> str:
        return self.root.name

    def summary(self) -> dict:
        photos = sum(len(s.get("photos", [])) for s in self.samples)
        env_bytes = sum(int(s.get("bytes") or 0) for s in self.shots)
        sample_bytes = sum(
            int(p.get("bytes") or 0) for s in self.samples for p in s.get("photos", [])
        )
        return {
            "id": self.id,
            "dir": str(self.root),
            "date": self.config.get("date"),
            "operator": self.config.get("operator", ""),
            "notes": self.config.get("notes", ""),
            "rooms": self.config.get("rooms", []),
            "created_at": self.config.get("created_at"),
            "sample_count": len(self.samples),
            "photo_count": photos,
            "env_count": len(self.shots),
            "bytes": env_bytes + sample_bytes,
            "next_index": self.config.get("next_index", 1),
        }

    # -- 持久化 -----------------------------------------------------------

    def save(self) -> None:
        _atomic_write(self.root / "session.json", json.dumps(self.config, ensure_ascii=False, indent=2))
        _atomic_write(
            self.root / "samples.json",
            json.dumps({"samples": self.samples}, ensure_ascii=False, indent=2),
        )
        _atomic_write(
            self.root / "environment.json",
            json.dumps({"shots": self.shots}, ensure_ascii=False, indent=2),
        )
        self._write_csv()

    def _write_csv(self) -> None:
        self._write_measurements()
        self._write_photos()
        self._write_environment()

    def _write_measurements(self) -> None:
        header = [
            "编号", "菇房", "棚架", "层", "生长形态", "簇内个数",
            "总高度_mm", "最大宽度_mm", "重量_g", "照片张数", "缺拍槽位",
            "备注", "采集时间",
        ]
        rows = []
        for sample in sorted(self.samples, key=lambda s: s.get("index", 0)):
            taken = {p.get("slot") for p in sample.get("photos", [])}
            missing = [
                naming.slot_label(key) for key in naming.SLOT_KEYS
                if key not in taken and naming.SLOT_BY_KEY[key]["required"]
            ]
            rows.append(
                [
                    sample.get("id"),
                    sample.get("room") or "",
                    sample.get("row") if sample.get("row") is not None else "",
                    sample.get("layer") if sample.get("layer") is not None else "",
                    MODE_LABELS.get(sample.get("mode"), sample.get("mode") or ""),
                    sample.get("cluster_count") if sample.get("cluster_count") is not None else "",
                    _num(sample.get("height_mm")),
                    _num(sample.get("width_mm")),
                    _num(sample.get("weight_g")),
                    len(sample.get("photos", [])),
                    "；".join(missing),
                    sample.get("notes") or "",
                    sample.get("created_at") or "",
                ]
            )
        _write_csv_rows(self.root / "measurements.csv", header, rows)

    def _write_photos(self) -> None:
        header = ["编号", "文件名", "槽位", "槽位名称", "来源", "字节数", "拍摄时间", "相对路径"]
        rows = []
        for sample in sorted(self.samples, key=lambda s: s.get("index", 0)):
            for photo in sample.get("photos", []):
                slot = photo.get("slot", "")
                rows.append(
                    [
                        sample.get("id"),
                        photo.get("name", ""),
                        slot,
                        naming.slot_label(slot) if slot in naming.SLOT_BY_KEY else slot,
                        photo.get("source", ""),
                        photo.get("bytes", ""),
                        photo.get("shot_at", ""),
                        photo.get("rel", ""),
                    ]
                )
        _write_csv_rows(self.root / "photos.csv", header, rows)

    def _write_environment(self) -> None:
        header = ["文件名", "菇房", "棚架", "层", "视角", "视角名称", "来源", "字节数", "拍摄时间", "相对路径"]
        rows = []
        for shot in self.shots:
            view = shot.get("view", "")
            rows.append(
                [
                    shot.get("name", ""),
                    shot.get("room", ""),
                    shot.get("row") if shot.get("row") is not None else "",
                    shot.get("layer") if shot.get("layer") is not None else "",
                    view,
                    naming.env_view_label(view),
                    shot.get("source", ""),
                    shot.get("bytes", ""),
                    shot.get("shot_at", ""),
                    shot.get("rel", ""),
                ]
            )
        _write_csv_rows(self.root / "environment.csv", header, rows)

    # -- 样本 -------------------------------------------------------------

    def get_sample(self, sid) -> dict:
        sid = naming.normalize_sample_id(sid)
        for sample in self.samples:
            if sample.get("id") == sid:
                return sample
        raise StoreError(f"找不到样本 {sid}")

    def add_sample(self, room=None, row=None, layer=None, mode="individual", cluster_count=None, notes="") -> dict:
        index = int(self.config.get("next_index") or 1)
        existing = {s.get("id") for s in self.samples}
        while naming.sample_id(index) in existing:
            index += 1
        sid = naming.sample_id(index)
        if mode not in MODE_LABELS:
            raise StoreError(f"未知的生长形态: {mode!r}")
        sample = {
            "id": sid,
            "index": index,
            "room": naming.slug(room, fallback="") or None,
            "row": _opt_int(row, "排号"),
            "layer": _opt_int(layer, "层号"),
            "mode": mode,
            "cluster_count": _opt_int(cluster_count, "簇内个数"),
            "height_mm": None,
            "width_mm": None,
            "weight_g": None,
            "notes": str(notes or ""),
            "created_at": _now(),
            "updated_at": _now(),
            "photos": [],
        }
        self.config["next_index"] = index + 1
        self.samples.append(sample)
        (self.root / naming.sample_dir(sid)).mkdir(parents=True, exist_ok=True)
        self.save()
        return sample

    def update_sample(self, sid, patch: dict) -> dict:
        sample = self.get_sample(sid)
        allowed = {
            "room", "row", "layer", "mode", "cluster_count",
            "height_mm", "width_mm", "weight_g", "notes",
        }
        for key, value in (patch or {}).items():
            if key not in allowed:
                continue
            if key == "room":
                sample["room"] = naming.slug(value, fallback="") or None
            elif key in ("row", "layer", "cluster_count"):
                sample[key] = _opt_int(value, key)
            elif key in ("height_mm", "width_mm", "weight_g"):
                sample[key] = _opt_float(value, key)
            elif key == "mode":
                if value not in MODE_LABELS:
                    raise StoreError(f"未知的生长形态: {value!r}")
                sample[key] = value
            else:
                sample[key] = str(value or "")
        sample["updated_at"] = _now()
        self.save()
        return sample

    def delete_sample(self, sid) -> str:
        sample = self.get_sample(sid)
        self.samples.remove(sample)
        self.save()
        return sample.get("id", sid)

    # -- 照片 -------------------------------------------------------------

    def add_sample_photo(self, sid, slot, data: bytes, ext: str, source: str = "phone") -> dict:
        sample = self.get_sample(sid)
        if slot not in naming.SLOT_BY_KEY:
            raise StoreError(f"未知的拍摄槽位: {slot!r}")
        seq = 1 + sum(1 for p in sample["photos"] if p.get("slot") == slot)
        rel = naming.sample_photo_rel(sid, slot, seq=seq, ext=ext)
        target = self.root / rel
        while target.exists():
            seq += 1
            rel = naming.sample_photo_rel(sid, slot, seq=seq, ext=ext)
            target = self.root / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        entry = self._photo_entry(target.name, rel, slot, len(data), source)
        sample["photos"].append(entry)
        sample["updated_at"] = _now()
        self.save()
        return entry

    def add_env_shot(self, room, view, data: bytes, *, row=None, layer=None, ext: str = ".jpg", source: str = "phone") -> dict:
        if view not in naming.ENV_VIEW_BY_KEY:
            raise StoreError(f"未知的环境视角: {view!r}")
        room_name = naming.slug(room, fallback="") or "未知菇房"
        scope = naming.ENV_VIEW_BY_KEY[view]["scope"]
        row = _opt_int(row, "排号") if scope in ("row", "bed") else None
        layer = _opt_int(layer, "层号") if scope == "bed" else None
        if scope in ("row", "bed") and row is None:
            raise StoreError("拍棚架或培养床时必须先选择排号")
        if scope == "bed" and layer is None:
            raise StoreError("拍培养床时必须先选择层号")

        seq = 1 + sum(
            1
            for shot in self.shots
            if shot.get("room") == room_name
            and shot.get("view") == view
            and shot.get("row") == row
            and shot.get("layer") == layer
        )
        rel = naming.env_photo_rel(room_name, view, row=row, layer=layer, seq=seq, ext=ext)
        target = self.root / rel
        while target.exists():
            seq += 1
            rel = naming.env_photo_rel(room_name, view, row=row, layer=layer, seq=seq, ext=ext)
            target = self.root / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)

        shot = {
            "name": target.name,
            "rel": rel,
            "room": room_name,
            "row": row,
            "layer": layer,
            "view": view,
            "source": source,
            "bytes": len(data),
            "shot_at": _now(),
        }
        self.shots.append(shot)
        self.save()
        return shot

    def add_sensor_files(self, sid, files) -> list:
        """登记 D435i 快照（文件已经写好，这里只做索引）。"""
        sample = self.get_sample(sid)
        entries = []
        for item in files:
            path = self.root / item["rel"]
            entry = self._photo_entry(path.name, item["rel"], f"d435i-{item['kind']}", path.stat().st_size, "d435i")
            sample["photos"] = [p for p in sample["photos"] if p.get("rel") != entry["rel"]]
            sample["photos"].append(entry)
            entries.append(entry)
        sample["updated_at"] = _now()
        self.save()
        return entries

    def delete_photo(self, rel: str) -> dict:
        rel = str(rel or "").strip().lstrip("/")
        if not rel:
            raise StoreError("缺少要删除的文件")
        for sample in self.samples:
            for photo in list(sample.get("photos", [])):
                if photo.get("rel") == rel:
                    sample["photos"].remove(photo)
                    sample["updated_at"] = _now()
                    self._unlink(rel)
                    self.save()
                    return {"removed": rel, "sample": sample.get("id")}
        for shot in list(self.shots):
            if shot.get("rel") == rel:
                self.shots.remove(shot)
                self._unlink(rel)
                self.save()
                return {"removed": rel, "sample": None}
        raise StoreError(f"索引中没有这个文件: {rel}")

    def _unlink(self, rel: str) -> None:
        target = (self.root / rel).resolve()
        try:
            target.relative_to(self.root.resolve())
        except ValueError:
            return
        try:
            target.unlink()
        except FileNotFoundError:
            pass

    @staticmethod
    def _photo_entry(name: str, rel: str, slot: str, size: int, source: str) -> dict:
        return {
            "name": name,
            "rel": rel,
            "slot": slot,
            "bytes": int(size),
            "source": source,
            "shot_at": _now(),
        }

    # -- 覆盖度 -----------------------------------------------------------

    def coverage(self) -> dict:
        rooms = []
        configured = {r["name"]: r for r in self.config.get("rooms", [])}
        names = list(configured)
        for shot in self.shots:
            if shot.get("room") not in names:
                names.append(shot.get("room"))
        for name in names:
            conf = configured.get(name, {"name": name, "rows": 0, "layers": 0})
            shots = [s for s in self.shots if s.get("room") == name]
            row_shots = sorted({s["row"] for s in shots if s.get("view") == "row" and s.get("row")})
            bed_shots = sorted(
                {(s["row"], s["layer"]) for s in shots if s.get("view") == "bed" and s.get("row") and s.get("layer")}
            )
            rooms.append(
                {
                    "name": name,
                    "rows": int(conf.get("rows") or 0),
                    "layers": int(conf.get("layers") or 0),
                    "overview": sum(1 for s in shots if s.get("view") == "overview"),
                    "row_shots": row_shots,
                    "bed_shots": [[r, l] for r, l in bed_shots],
                    "photos": len(shots),
                }
            )

        per_slot = {key: 0 for key in naming.SLOT_KEYS}
        complete = 0
        for sample in self.samples:
            taken = {p.get("slot") for p in sample.get("photos", [])}
            for key in taken:
                if key in per_slot:
                    per_slot[key] += 1
            if all(key in taken for key in naming.REQUIRED_SLOTS):
                complete += 1

        return {
            "rooms": rooms,
            "samples": {
                "total": len(self.samples),
                "complete": complete,
                "per_slot": per_slot,
            },
        }


class Store:
    """数据根目录 + 当前会话。"""

    def __init__(self, root, session_id=None):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._session = None
        with self._lock:
            candidate = session_id or self._read_current()
            if candidate:
                try:
                    self._session = Session(self.root / candidate)
                except StoreError:
                    self._session = None
            if self._session is None:
                sessions = self.list_sessions()
                if sessions:
                    self._session = Session(Path(sessions[-1]["dir"]))

    # -- 会话管理 ---------------------------------------------------------

    def _read_current(self):
        path = self.root / "current.txt"
        try:
            return path.read_text(encoding="utf-8").strip() or None
        except OSError:
            return None

    def _write_current(self, session_id: str) -> None:
        _atomic_write(self.root / "current.txt", session_id + "\n")

    def list_sessions(self) -> list:
        found = []
        for child in sorted(self.root.iterdir()):
            if not child.is_dir():
                continue
            if not (child / "session.json").is_file():
                continue
            try:
                found.append(Session(child).summary())
            except StoreError:
                continue
        return found

    @property
    def current(self):
        return self._session

    def require(self) -> Session:
        with self._lock:
            if self._session is None:
                raise StoreError("还没有采集会话，请先在“设置”里新建一个会话")
            return self._session

    def create(self, date=None, operator="", rooms=None, notes="", suffix="") -> Session:
        with self._lock:
            stamp = re.sub(r"\D", "", str(date or ""))[:8] or datetime.now().strftime("%Y%m%d")
            base = f"grass_mushroom_{stamp}"
            extra = naming.slug(suffix, fallback="")
            if extra:
                base += f"-{extra}"
            session_id = base
            counter = 2
            while (self.root / session_id).exists():
                session_id = f"{base}-{counter}"
                counter += 1
            root = self.root / session_id
            root.mkdir(parents=True, exist_ok=True)
            for sub in ("samples", "environment"):
                (root / sub).mkdir(parents=True, exist_ok=True)
            config = {
                "id": session_id,
                "date": stamp,
                "operator": str(operator or ""),
                "notes": str(notes or ""),
                "rooms": parse_rooms(rooms) if rooms is not None else [],
                "created_at": _now(),
                "next_index": 1,
            }
            _atomic_write(root / "session.json", json.dumps(config, ensure_ascii=False, indent=2))
            _atomic_write(root / "samples.json", json.dumps({"samples": []}, ensure_ascii=False, indent=2))
            _atomic_write(root / "environment.json", json.dumps({"shots": []}, ensure_ascii=False, indent=2))
            self._session = Session(root)
            self._session.save()
            self._write_current(session_id)
            return self._session

    def open(self, session_id: str) -> Session:
        with self._lock:
            root = (self.root / str(session_id)).resolve()
            try:
                root.relative_to(self.root)
            except ValueError as exc:
                raise StoreError("会话路径不合法") from exc
            if not (root / "session.json").is_file():
                raise StoreError(f"找不到会话 {session_id}")
            self._session = Session(root)
            self._write_current(self._session.id)
            return self._session

    def update_config(self, patch: dict) -> dict:
        with self._lock:
            session = self.require()
            if "operator" in patch:
                session.config["operator"] = str(patch["operator"] or "")
            if "notes" in patch:
                session.config["notes"] = str(patch["notes"] or "")
            if "date" in patch:
                stamp = re.sub(r"\D", "", str(patch["date"] or ""))[:8]
                if stamp:
                    session.config["date"] = stamp
            if "rooms" in patch:
                session.config["rooms"] = parse_rooms(patch["rooms"])
            session.save()
            return session.summary()

    # -- 转发 -------------------------------------------------------------

    def add_sample(self, **kwargs) -> dict:
        with self._lock:
            return self.require().add_sample(**kwargs)

    def update_sample(self, sid, patch) -> dict:
        with self._lock:
            return self.require().update_sample(sid, patch)

    def delete_sample(self, sid) -> str:
        with self._lock:
            return self.require().delete_sample(sid)

    def add_sample_photo(self, sid, slot, data, ext, source="phone") -> dict:
        with self._lock:
            return self.require().add_sample_photo(sid, slot, data, ext, source)

    def add_env_shot(self, room, view, data, **kwargs) -> dict:
        with self._lock:
            return self.require().add_env_shot(room, view, data, **kwargs)

    def add_sensor_files(self, sid, files) -> list:
        with self._lock:
            return self.require().add_sensor_files(sid, files)

    def delete_photo(self, rel) -> dict:
        with self._lock:
            return self.require().delete_photo(rel)

    def resolve_media(self, rel: str) -> Path:
        """把会话内相对路径解析成绝对路径，并阻止目录穿越。"""
        with self._lock:
            session = self.require()
            target = (session.root / str(rel).lstrip("/")).resolve()
            try:
                target.relative_to(session.root.resolve())
            except ValueError as exc:
                raise StoreError("路径不合法") from exc
            return target


def _num(value):
    """CSV 单元格：整数不带小数尾巴，空值留空。"""
    if value is None:
        return ""
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


def _write_csv_rows(path: Path, header, rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)
    os.replace(tmp, path)
