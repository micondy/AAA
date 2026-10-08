#!/usr/bin/env python3
"""Fail when a ROS package is missing from the maintained package catalog."""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent
CATALOG = ROOT / "docs" / "architecture" / "PACKAGE_CATALOG.md"
SEARCH_ROOTS = (
    PROJECT_ROOT / "mobile_base_ws" / "src",
    ROOT / "src" / "robot_research",
    ROOT / "src" / "vendor",
)


def package_names() -> list[str]:
    names: set[str] = set()
    for search_root in SEARCH_ROOTS:
        for manifest in search_root.rglob("package.xml"):
            if any(part in {"build", "install", "log"} for part in manifest.parts):
                continue
            name = ET.parse(manifest).getroot().findtext("name")
            if name:
                names.add(name.strip())
    return sorted(names)


def main() -> int:
    catalog = CATALOG.read_text(encoding="utf-8")
    missing = [name for name in package_names() if f"`{name}`" not in catalog]
    if missing:
        print("架构包目录缺少：" + ", ".join(missing), file=sys.stderr)
        return 1
    print(f"架构包目录完整：{len(package_names())} 个 ROS 包均已收录")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
