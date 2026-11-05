"""存储层测试：目录结构、照片命名、CSV 导出与覆盖度统计。"""

import csv
import json
from pathlib import Path

import pytest

from gmkit import naming
from gmkit.store import Store, StoreError, parse_rooms

JPEG = b"\xff\xd8\xff\xe0" + b"0" * 64


@pytest.fixture()
def store(tmp_path):
    return Store(tmp_path / "data")


@pytest.fixture()
def session(store):
    return store.create(date="20261105", operator="张三", rooms="A,4,3\nB,2,2", notes="首日")


def read_csv(path: Path):
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def test_create_session_layout(session):
    root = session.root
    assert root.name == "grass_mushroom_20261105"
    assert (root / "session.json").is_file()
    assert (root / "measurements.csv").is_file()
    assert (root / "samples").is_dir()
    assert (root / "environment").is_dir()
    assert session.summary()["rooms"] == [
        {"name": "A", "rows": 4, "layers": 3},
        {"name": "B", "rows": 2, "layers": 2},
    ]


def test_parse_rooms_variants():
    assert parse_rooms("A,12,3") == [{"name": "A", "rows": 12, "layers": 3}]
    assert parse_rooms("A 房，12，3\nB") == [
        {"name": "A-房", "rows": 12, "layers": 3},
        {"name": "B", "rows": 0, "layers": 0},
    ]
    assert parse_rooms("") == []


def test_sample_ids_increment_and_skip_used(session):
    first = session.add_sample(room="A", row=1, layer=1)
    second = session.add_sample(room="A", row=1, layer=2)
    assert first["id"] == "S001"
    assert second["id"] == "S002"
    assert (session.root / "samples/S001").is_dir()


def test_photo_naming_and_duplicates(session):
    session.add_sample(room="A")
    entry = session.add_sample_photo("S001", "inplace-far", JPEG, ".jpg")
    assert entry["name"] == "s001-inplace-far.jpg"
    assert entry["rel"] == "samples/S001/s001-inplace-far.jpg"
    assert (session.root / entry["rel"]).read_bytes() == JPEG

    again = session.add_sample_photo("S001", "inplace-far", JPEG, ".jpg")
    assert again["name"] == "s001-inplace-far-2.jpg"
    assert len(session.get_sample("S001")["photos"]) == 2


def test_env_shot_naming_and_scope(session):
    shot = session.add_env_shot("A", "bed", JPEG, row=2, layer=1)
    assert shot["name"] == "env-A-r02-l01-bed.jpg"
    assert shot["rel"] == "environment/A/env-A-r02-l01-bed.jpg"

    overview = session.add_env_shot("A", "overview", JPEG)
    assert overview["name"] == "env-A-overview.jpg"

    with pytest.raises(StoreError):
        session.add_env_shot("A", "bed", JPEG, layer=1)


def test_measurements_export(session):
    session.add_sample(room="A", row=1, layer=1)
    session.update_sample("S001", {"height_mm": "35.4", "width_mm": 22, "weight_g": "18.6", "notes": "破损"})
    rows = read_csv(session.root / "measurements.csv")
    assert len(rows) == 1
    row = rows[0]
    assert row["编号"] == "S001"
    assert row["总高度_mm"] == "35.4"
    assert row["最大宽度_mm"] == "22"
    assert row["重量_g"] == "18.6"
    assert row["生长形态"] == "单个"
    assert "原位远照" in row["缺拍槽位"]


def test_photo_csv_tracks_slots(session):
    session.add_sample(room="A")
    session.add_sample_photo("S001", "inplace-near", JPEG, ".jpg")
    rows = read_csv(session.root / "photos.csv")
    assert rows[0]["文件名"] == "s001-inplace-near.jpg"
    assert rows[0]["槽位名称"] == "原位近照"


def test_coverage_reports_missing_beds(session):
    session.add_env_shot("A", "overview", JPEG)
    session.add_env_shot("A", "row", JPEG, row=1)
    session.add_env_shot("A", "bed", JPEG, row=1, layer=1)
    cov = session.coverage()
    room_a = next(r for r in cov["rooms"] if r["name"] == "A")
    assert room_a["rows"] == 4 and room_a["layers"] == 3
    assert room_a["overview"] == 1
    assert room_a["row_shots"] == [1]
    assert room_a["bed_shots"] == [[1, 1]]


def test_complete_sample_counting(session):
    session.add_sample(room="A")
    for slot in naming.REQUIRED_SLOTS:
        session.add_sample_photo("S001", slot, JPEG, ".jpg")
    assert session.coverage()["samples"]["complete"] == 1


def test_delete_photo_removes_file_and_index(session):
    session.add_sample(room="A")
    entry = session.add_sample_photo("S001", "picked-top", JPEG, ".jpg")
    session.delete_photo(entry["rel"])
    assert not (session.root / entry["rel"]).exists()
    assert session.get_sample("S001")["photos"] == []


def test_delete_sample_keeps_files(session):
    session.add_sample(room="A")
    entry = session.add_sample_photo("S001", "picked-top", JPEG, ".jpg")
    session.delete_sample("S001")
    assert session.samples == []
    assert (session.root / entry["rel"]).exists()


def test_resolve_media_blocks_traversal(store, session):
    with pytest.raises(StoreError):
        store.resolve_media("../../etc/passwd")


def test_invalid_measurement_rejected(session):
    session.add_sample(room="A")
    with pytest.raises(StoreError):
        session.update_sample("S001", {"weight_g": "abc"})
    with pytest.raises(StoreError):
        session.update_sample("S001", {"height_mm": -3})


def test_current_session_persisted(tmp_path):
    data = tmp_path / "data"
    first = Store(data).create(date="20261101")
    second = Store(data).create(date="20261102")
    assert first.id != second.id
    reopened = Store(data)
    assert reopened.current.id == second.id
    assert [s["id"] for s in reopened.list_sessions()] == [first.id, second.id]


def test_sensor_files_registered_once(session):
    session.add_sample(room="A")
    files = [
        {"kind": "color", "rel": naming.d435i_rel("S001", "color"), "bytes": 10},
        {"kind": "depth", "rel": naming.d435i_rel("S001", "depth"), "bytes": 10},
    ]
    for item in files:
        (session.root / item["rel"]).parent.mkdir(parents=True, exist_ok=True)
        (session.root / item["rel"]).write_bytes(b"x")
    session.add_sensor_files("S001", files)
    session.add_sensor_files("S001", files)
    slots = [p["slot"] for p in session.get_sample("S001")["photos"]]
    assert slots.count("d435i-color") == 1
    assert slots.count("d435i-depth") == 1


def test_session_json_is_valid_utf8(session):
    session.add_sample(room="A房")
    payload = json.loads((session.root / "samples.json").read_text(encoding="utf-8"))
    assert payload["samples"][0]["room"] == "A房"
