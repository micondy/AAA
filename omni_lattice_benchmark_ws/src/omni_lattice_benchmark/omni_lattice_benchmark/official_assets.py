#!/usr/bin/env python3
"""Fetch the pinned official Nav2 Jazzy Omni sample lattice."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
from urllib.request import urlopen


NAV2_COMMIT = "426682ff88405a99617294f5cc8401a61aa5fa50"
SHA256 = "814733962e71376dada74193b7aa911b91f0c71b2b867f83994a6eefc0d0ee4b"
URL = (
    "https://raw.githubusercontent.com/ros-navigation/navigation2/"
    f"{NAV2_COMMIT}/nav2_smac_planner/lattice_primitives/sample_primitives/"
    "5cm_resolution/0.5m_turning_radius/omni/output.json"
)


def fetch(target: Path) -> None:
    with urlopen(URL, timeout=30) as response:
        payload = response.read()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != SHA256:
        raise RuntimeError(f"Official lattice checksum mismatch: {digest}")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(payload)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("lattice/nav2_official_omni_16_5cm_r0.5.json"),
    )
    args = parser.parse_args()
    fetch(args.output)
    print(f"Fetched pinned Nav2 Omni lattice to {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
