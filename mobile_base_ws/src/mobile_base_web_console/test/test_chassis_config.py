from pathlib import Path

import pytest

from mobile_base_web_console.chassis_config import ChassisConfigManager


def test_default_config_is_valid(monkeypatch, tmp_path):
    monkeypatch.setenv("MOBILE_BASE_CONFIG_DIR", str(tmp_path))
    manager = ChassisConfigManager(
        Path(__file__).parents[1] / "config" / "chassis.yaml"
    )
    assert list(manager.get()["wheels"]) == [
        "front_left", "front_right", "rear_left", "rear_right"
    ]
    assert manager.get()["geometry"]["body_length"] == pytest.approx(0.62)
    assert manager.get()["geometry"]["body_width"] == pytest.approx(0.58)
    assert manager.get()["geometry"]["wheel_width"] == pytest.approx(0.0683)
    assert manager.get()["geometry"]["wheel_center_offset"] == pytest.approx(0.043)
    assert manager.get()["geometry"]["footprint_padding"] == pytest.approx(0.02)


def test_save_creates_backup(monkeypatch, tmp_path):
    monkeypatch.setenv("MOBILE_BASE_CONFIG_DIR", str(tmp_path))
    manager = ChassisConfigManager(
        Path(__file__).parents[1] / "config" / "chassis.yaml"
    )
    assert manager.save(manager.get())["backup"] is None
    assert Path(manager.save(manager.get())["backup"]).is_file()


def test_rejects_duplicate_motor_id(monkeypatch, tmp_path):
    monkeypatch.setenv("MOBILE_BASE_CONFIG_DIR", str(tmp_path))
    manager = ChassisConfigManager(
        Path(__file__).parents[1] / "config" / "chassis.yaml"
    )
    invalid = manager.get()
    invalid["wheels"]["rear_right"]["steering"]["motor_id"] = 1
    with pytest.raises(ValueError, match="motor_id 重复"):
        manager.save(invalid)


def test_rejects_duplicate_zlac_channel(monkeypatch, tmp_path):
    monkeypatch.setenv("MOBILE_BASE_CONFIG_DIR", str(tmp_path))
    manager = ChassisConfigManager(
        Path(__file__).parents[1] / "config" / "chassis.yaml"
    )
    invalid = manager.get()
    existing = invalid["wheels"]["front_left"]["drive"]
    invalid["wheels"]["rear_right"]["drive"].update(
        node_id=existing["node_id"], channel=existing["channel"]
    )
    with pytest.raises(ValueError, match="ZLAC 节点/通道重复"):
        manager.save(invalid)


def test_visualization_draws_physical_and_navigation_footprints():
    web_root = Path(__file__).parents[1] / "web"
    script = (web_root / "app.js").read_text(encoding="utf-8")
    page = (web_root / "index.html").read_text(encoding="utf-8")
    assert "drawRobotFootprint" in script
    assert "drawSceneRobot" in script
    assert "drawChassisTopView" in script
    assert "wheelCommandVector" in script
    assert "实际滚动方向" in page
    assert "驱动映射" in script
    assert "activateMotionPreset" in script
    assert "动作按钮单击即执行" in page
    assert "wheel_center_offset" in script
    assert "footprint_padding" in script
    assert 'id="bodyLegend"' in page
