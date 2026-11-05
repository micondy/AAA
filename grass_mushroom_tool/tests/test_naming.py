"""命名规则单元测试：文件名必须永远能反推回“哪个样本、哪个部位”。"""

import pytest

from gmkit import naming


def test_sample_id_padding():
    assert naming.sample_id(1) == "S001"
    assert naming.sample_id(42) == "S042"
    assert naming.sample_id(1234) == "S1234"


@pytest.mark.parametrize("value,expected", [("7", "S007"), ("007", "S007"), ("s7", "S007"), ("S007", "S007")])
def test_normalize_sample_id(value, expected):
    assert naming.normalize_sample_id(value) == expected


def test_sample_photo_names():
    assert naming.sample_photo("S007", "inplace-far") == "s007-inplace-far.jpg"
    assert naming.sample_photo("S007", "picked-top", seq=2) == "s007-picked-top-2.jpg"
    assert naming.sample_photo_rel("S007", "picked-bottom", ext=".png") == "samples/S007/s007-picked-bottom.png"


def test_sample_photo_rejects_unknown_slot():
    with pytest.raises(naming.NamingError):
        naming.sample_photo("S001", "unknown-slot")


def test_env_photo_scope():
    assert naming.env_photo("A", "overview") == "env-A-overview.jpg"
    assert naming.env_photo("A", "row", row=2) == "env-A-r02-row.jpg"
    assert naming.env_photo("A", "bed", row=2, layer=1) == "env-A-r02-l01-bed.jpg"
    assert naming.env_photo("A", "bed", row=12, layer=3, seq=2) == "env-A-r12-l03-bed-2.jpg"


def test_env_photo_requires_row_and_layer():
    with pytest.raises(naming.NamingError):
        naming.env_photo("A", "bed", layer=1)
    with pytest.raises(naming.NamingError):
        naming.env_photo("A", "row")


def test_env_photo_rel_uses_room_folder():
    assert naming.env_photo_rel("A", "overview") == "environment/A/env-A-overview.jpg"


def test_d435i_file_names():
    assert naming.d435i_file("S007", "color") == "s007-d435i-color.png"
    assert naming.d435i_file("S007", "points") == "s007-d435i-points.ply"
    assert naming.d435i_rel("S007", "meta") == "samples/S007/s007-d435i-meta.json"


def test_parse_sample_photo_roundtrip():
    for slot in naming.SLOT_KEYS:
        name = naming.sample_photo("S012", slot, seq=3)
        parsed = naming.parse_sample_photo(name)
        assert parsed["id"] == "S012"
        assert parsed["slot"] == slot
        assert parsed["seq"] == 3


def test_parse_env_photo_roundtrip():
    name = naming.env_photo("A", "bed", row=3, layer=2, seq=4)
    parsed = naming.parse_env_photo(name)
    assert parsed["room"] == "A"
    assert parsed["row"] == 3
    assert parsed["layer"] == 2
    assert parsed["view"] == "bed"
    assert parsed["seq"] == 4


def test_slug_keeps_chinese_and_drops_spaces():
    assert naming.slug("A 房") == "A-房"
    assert naming.slug("  1号/东侧  ") == "1号-东侧"
    assert naming.slug("") == "x"
    assert naming.slug(None) == "x"


def test_required_slots_match_plan():
    assert naming.REQUIRED_SLOTS == (
        "inplace-far",
        "inplace-near",
        "picked-top",
        "picked-back",
        "picked-bottom",
        "picked-weight",
    )
