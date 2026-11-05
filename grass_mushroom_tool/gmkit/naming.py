"""文件名与编号规则 —— 整个工具唯一的命名来源。

设计目标
--------
1. 一张照片的文件名本身就能说明“哪个样本、哪个阶段、哪个部位”，
   永远不会出现 ``IMG_0421.jpg`` / ``微信图片_20261105.jpg`` 这种无法对应的名字。
2. 顺序固定（与现场方案一致）：
   原位远照 → 原位近照 →（左/右补充）→ 采摘后整簇 → 正面 → 反面 → 底部
   →（簇内个体）→ 标尺照 → 秤读数。
3. 文件名只由 ASCII 字母、数字、中文、短横线、下划线组成，不含空格，
   在 Linux / Windows / 手机之间拷贝都不会出问题。

命名格式
--------
样本照片::

    s007-inplace-far.jpg
    s007-picked-top-2.jpg        # 同一槽位重拍时按序号递增

环境照片::

    env-A-overview.jpg           # 菇房全景
    env-A-r02-row.jpg            # A 房第 2 排棚架
    env-A-r02-l01-bed.jpg        # A 房第 2 排第 1 层培养床

D435i 数据::

    s007-d435i-color.png
    s007-d435i-depth.png
    s007-d435i-points.ply
    s007-d435i-meta.json
"""

from __future__ import annotations

import re
import unicodedata

# --------------------------------------------------------------------------
# 常量
# --------------------------------------------------------------------------

UNKNOWN = "x"

#: 样本拍摄槽位，顺序即现场操作顺序
SLOTS = (
    {
        "key": "inplace-far",
        "label": "原位远照",
        "hint": "稍远一张，保留所在位置和周围环境",
        "group": "原位",
        "required": True,
    },
    {
        "key": "inplace-near",
        "label": "原位近照",
        "hint": "近距离一张，草菇清晰完整",
        "group": "原位",
        "required": True,
    },
    {
        "key": "inplace-left",
        "label": "左向补充",
        "hint": "密集、大小混合或遮挡严重时补拍",
        "group": "原位",
        "required": False,
    },
    {
        "key": "inplace-right",
        "label": "右向补充",
        "hint": "密集、大小混合或遮挡严重时补拍",
        "group": "原位",
        "required": False,
    },
    {
        "key": "picked-cluster",
        "label": "采摘后整簇",
        "hint": "成簇样本先拍完整菌簇",
        "group": "采摘后",
        "required": False,
    },
    {
        "key": "picked-top",
        "label": "正面",
        "hint": "编号牌 + 直尺，完整正面",
        "group": "采摘后",
        "required": True,
    },
    {
        "key": "picked-back",
        "label": "反面",
        "hint": "翻转后拍另一面",
        "group": "采摘后",
        "required": True,
    },
    {
        "key": "picked-bottom",
        "label": "底部",
        "hint": "底部、菌柄和菌托",
        "group": "采摘后",
        "required": True,
    },
    {
        "key": "picked-member",
        "label": "簇内个体",
        "hint": "成簇样本分开后逐个拍，可多次",
        "group": "采摘后",
        "required": False,
    },
    {
        "key": "picked-ruler",
        "label": "标尺特写",
        "hint": "编号牌与直尺同框的测量参照",
        "group": "采摘后",
        "required": False,
    },
    {
        "key": "picked-weight",
        "label": "秤读数",
        "hint": "电子秤数字清晰可读",
        "group": "采摘后",
        "required": True,
    },
)

SLOT_KEYS = tuple(slot["key"] for slot in SLOTS)
SLOT_BY_KEY = {slot["key"]: slot for slot in SLOTS}
REQUIRED_SLOTS = tuple(slot["key"] for slot in SLOTS if slot["required"])

#: 环境视角，``scope`` 决定需要填写排/层
ENV_VIEWS = (
    {
        "key": "overview",
        "label": "菇房全景",
        "hint": "每个菇房一张，建立整体印象",
        "scope": "room",
        "required": True,
    },
    {
        "key": "row",
        "label": "棚架全排",
        "hint": "每排棚架拍一张",
        "scope": "row",
        "required": True,
    },
    {
        "key": "bed",
        "label": "培养床层",
        "hint": "每层培养床拍一张，含有无菇区域",
        "scope": "bed",
        "required": True,
    },
    {
        "key": "empty-bed",
        "label": "空床",
        "hint": "空床状态与床面材质",
        "scope": "bed",
        "required": False,
    },
    {
        "key": "aisle",
        "label": "通道",
        "hint": "通道宽度与地面情况",
        "scope": "room",
        "required": False,
    },
    {
        "key": "site",
        "label": "现场环境",
        "hint": "温湿度、设备、外景等",
        "scope": "room",
        "required": False,
    },
)

ENV_VIEW_KEYS = tuple(view["key"] for view in ENV_VIEWS)
ENV_VIEW_BY_KEY = {view["key"]: view for view in ENV_VIEWS}

#: D435i 快照类型 -> 扩展名
D435I_KINDS = {
    "color": ".png",
    "depth": ".png",
    "points": ".ply",
    "meta": ".json",
}

D435I_SLOTS = tuple(f"d435i-{kind}" for kind in D435I_KINDS)

_SAFE_RE = re.compile(r"[^0-9A-Za-z\u4e00-\u9fff_-]+")


class NamingError(ValueError):
    """命名参数不合法。"""


# --------------------------------------------------------------------------
# 基础工具
# --------------------------------------------------------------------------


def slug(value, *, fallback: str = UNKNOWN) -> str:
    """把任意文本压成文件名安全的片段（保留中文，方便人读）。"""
    text = unicodedata.normalize("NFKC", str(value if value is not None else "")).strip()
    text = _SAFE_RE.sub("-", text)
    text = re.sub(r"-{2,}", "-", text).strip("-_")
    return text[:32] or fallback


def sample_id(index) -> str:
    """1 -> ``S001``。"""
    try:
        number = int(index)
    except (TypeError, ValueError) as exc:
        raise NamingError(f"样本序号不是整数: {index!r}") from exc
    if number < 1:
        raise NamingError(f"样本序号必须大于 0: {number}")
    return f"S{number:03d}"


def normalize_sample_id(value) -> str:
    """把 ``7`` / ``007`` / ``s7`` / ``S007`` 统一成 ``S007``。"""
    text = str(value or "").strip()
    if not text:
        raise NamingError("样本编号为空")
    match = re.fullmatch(r"[Ss]?0*(\d+)", text)
    if not match:
        raise NamingError(f"无法识别的样本编号: {value!r}")
    return sample_id(match.group(1))


def sample_dir(sid) -> str:
    """样本照片所在目录（会话内相对路径）。"""
    return f"samples/{normalize_sample_id(sid)}"


def _seq_suffix(seq: int) -> str:
    return "" if int(seq) <= 1 else f"-{int(seq)}"


def sample_photo(sid, slot, *, seq: int = 1, ext: str = ".jpg") -> str:
    """``S007`` + ``inplace-far`` -> ``s007-inplace-far.jpg``。"""
    if slot not in SLOT_BY_KEY:
        raise NamingError(f"未知的拍摄槽位: {slot!r}")
    sid = normalize_sample_id(sid)
    return f"{sid.lower()}-{slot}{_seq_suffix(seq)}{ext}"


def sample_photo_rel(sid, slot, *, seq: int = 1, ext: str = ".jpg") -> str:
    return f"{sample_dir(sid)}/{sample_photo(sid, slot, seq=seq, ext=ext)}"


def env_dir(room) -> str:
    return f"environment/{slug(room)}"


def env_photo(room, view, *, row=None, layer=None, seq: int = 1, ext: str = ".jpg") -> str:
    """``env-A-r02-l01-bed.jpg``，未使用的层级自动省略。"""
    if view not in ENV_VIEW_BY_KEY:
        raise NamingError(f"未知的环境视角: {view!r}")
    scope = ENV_VIEW_BY_KEY[view]["scope"]
    parts = ["env", slug(room)]
    if scope in ("row", "bed"):
        parts.append(f"r{_int_field(row, '排'):02d}")
    if scope == "bed":
        parts.append(f"l{_int_field(layer, '层'):02d}")
    parts.append(view)
    name = "-".join(parts)
    return f"{name}{_seq_suffix(seq)}{ext}"


def env_photo_rel(room, view, *, row=None, layer=None, seq: int = 1, ext: str = ".jpg") -> str:
    return f"{env_dir(room)}/{env_photo(room, view, row=row, layer=layer, seq=seq, ext=ext)}"


def d435i_file(sid, kind: str, *, seq: int = 1) -> str:
    """``S007`` + ``color`` -> ``s007-d435i-color.png``；第二组起加 ``-2``。"""
    if kind not in D435I_KINDS:
        raise NamingError(f"未知的 D435i 数据类型: {kind!r}")
    return f"{normalize_sample_id(sid).lower()}-d435i-{kind}{_seq_suffix(seq)}{D435I_KINDS[kind]}"


def d435i_rel(sid, kind: str, *, seq: int = 1) -> str:
    return f"{sample_dir(sid)}/{d435i_file(sid, kind, seq=seq)}"


def _int_field(value, label: str) -> int:
    if value is None or value == "":
        raise NamingError(f"{label}号不能为空")
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise NamingError(f"{label}号必须是整数: {value!r}") from exc
    if number < 0 or number > 99:
        raise NamingError(f"{label}号超出范围 0-99: {number}")
    return number


# --------------------------------------------------------------------------
# 反向解析（供后续标注脚本使用）
# --------------------------------------------------------------------------

_SAMPLE_PHOTO_RE = re.compile(r"^(s\d{3,})-(.+?)(?:-(\d+))?(\.[A-Za-z0-9]+)$")
_ENV_PHOTO_RE = re.compile(
    r"^env-(?P<room>.+?)(?:-r(?P<row>\d{2}))?(?:-l(?P<layer>\d{2}))?"
    r"-(?P<view>[a-z-]+?)(?:-(?P<seq>\d+))?(?P<ext>\.[A-Za-z0-9]+)$"
)


def parse_sample_photo(name: str) -> dict:
    """``s007-inplace-far-2.jpg`` -> ``{id, slot, seq, ext}``。"""
    match = _SAMPLE_PHOTO_RE.match(str(name))
    if not match:
        raise NamingError(f"不是样本照片文件名: {name!r}")
    sid, body, seq, ext = match.groups()
    candidates = [body] if body in SLOT_BY_KEY else []
    if not candidates:
        candidates = [key for key in SLOT_KEYS if body.startswith(key)]
        candidates.sort(key=len, reverse=True)
    if not candidates:
        raise NamingError(f"无法识别拍摄槽位: {name!r}")
    return {
        "id": normalize_sample_id(sid),
        "slot": candidates[0],
        "seq": int(seq) if seq else 1,
        "ext": ext,
    }


def parse_env_photo(name: str) -> dict:
    """``env-A-r02-l01-bed.jpg`` -> ``{room, row, layer, view, seq, ext}``。"""
    match = _ENV_PHOTO_RE.match(str(name))
    if not match:
        raise NamingError(f"不是环境照片文件名: {name!r}")
    groups = match.groupdict()
    if groups["view"] not in ENV_VIEW_BY_KEY:
        raise NamingError(f"无法识别环境视角: {name!r}")
    return {
        "room": groups["room"],
        "row": int(groups["row"]) if groups["row"] else None,
        "layer": int(groups["layer"]) if groups["layer"] else None,
        "view": groups["view"],
        "seq": int(groups["seq"]) if groups["seq"] else 1,
        "ext": groups["ext"],
    }


def slot_label(key: str) -> str:
    return SLOT_BY_KEY.get(key, {}).get("label", key)


def env_view_label(key: str) -> str:
    return ENV_VIEW_BY_KEY.get(key, {}).get("label", key)


def next_d435i_seq(session_root, sid):
    """按磁盘已有最大组号续号，删除中间组不会复用后续文件名。"""
    from pathlib import Path
    sid = normalize_sample_id(sid)
    folder = Path(session_root) / sample_dir(sid)
    pattern = re.compile(rf"{sid.lower()}-d435i-(?:color|depth|meta|points)(?:-([1-9][0-9]*))?\.(?:png|json|ply)$")
    groups = [int(match.group(1) or 1) for path in folder.glob(f'{sid.lower()}-d435i-*')
              if (match := pattern.fullmatch(path.name))]
    return max(groups, default=0) + 1
