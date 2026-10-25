import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from mobile_base_web_console.web_console import WebConsoleNode, _project_racks, _transform_point
from tf2_ros import TransformException


def transform(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
    ))


MODEL = {
    "schema": "rack_map/v1", "frame_id": "sim_world", "racks": [{
        "stable_id": "rack_0001", "position": [1.0, 0.0, 0.8],
        "layers": [{"stable_id": "layer_0001", "level_number": 1,
                    "center": [1.0, 0.0, 0.55], "observed": False,
                    "boundary": [[0.6, -0.8, 0.55], [1.4, -0.8, 0.55],
                                 [1.4, 0.8, 0.55], [0.6, 0.8, 0.55]],
                    "front_center": [0.6, 0.0, 0.55],
                    "front_normal": [-1.0, 0.0], "axis": [0.0, 1.0],
                    "observed_ratio": 0.6, "confidence": 0.8}],
    }],
}


def test_rack_points_follow_full_tf_and_keep_stable_ids():
    tf = transform(2.0, 3.0, 0.2, math.pi / 2)
    assert _transform_point([1.0, 0.0, 0.55], tf) == pytest.approx([2.0, 4.0, 0.75])
    racks = _project_racks(MODEL, tf)
    assert racks[0]["stable_id"] == "rack_0001"
    assert racks[0]["position"] == pytest.approx([2.0, 4.0, 1.0])
    assert racks[0]["layers"][0]["stable_id"] == "layer_0001"
    assert racks[0]["layers"][0]["observed"] is False
    layer = racks[0]["layers"][0]
    assert layer["boundary"][0] == pytest.approx([2.8, 3.6, 0.75])
    assert layer["front_center"] == pytest.approx([2.0, 3.6, 0.75])
    assert layer["axis"] == pytest.approx([-1.0, 0.0])
    assert layer["front_normal"] == pytest.approx([0.0, -1.0])
    assert layer["observed_ratio"] == pytest.approx(0.6)
    assert layer["confidence"] == pytest.approx(0.8)


def test_rack_snapshot_uses_map_and_mesh_frames_separately():
    calls = []

    def lookup(target, source, _stamp):
        calls.append((target, source))
        return transform(x=10.0 if target == "map" else 20.0)

    dummy = SimpleNamespace(
        lock=threading.RLock(), rack_model=MODEL, last_rack_time=time.monotonic(),
        map_frame="map", mesh_frame="odom",
        tf_buffer=SimpleNamespace(lookup_transform=lookup),
    )
    snapshot = WebConsoleNode.rack_snapshot(dummy)
    assert calls == [("map", "sim_world"), ("odom", "sim_world")]
    assert snapshot["map_racks"][0]["position"][0] == pytest.approx(11.0)
    assert snapshot["scene_racks"][0]["position"][0] == pytest.approx(21.0)


def test_rack_snapshot_skips_overlay_until_tf_exists():
    def missing_tf(_target, _source, _stamp):
        raise TransformException("not connected")

    dummy = SimpleNamespace(
        lock=threading.RLock(), rack_model=MODEL, last_rack_time=time.monotonic(),
        map_frame="map", mesh_frame="odom",
        tf_buffer=SimpleNamespace(lookup_transform=missing_tf),
    )
    snapshot = WebConsoleNode.rack_snapshot(dummy)
    assert len(snapshot["racks"]) == 1
    assert snapshot["map_racks"] == []
    assert snapshot["scene_racks"] == []


def test_semantic_scene_does_not_require_nvblox_mesh():
    dummy = SimpleNamespace(
        lock=threading.RLock(), rack_model=MODEL, last_rack_time=time.monotonic(),
        map_frame="", mesh_frame="",
        tf_buffer=SimpleNamespace(lookup_transform=lambda target, source, _stamp: transform(x=2.0)),
    )
    snapshot = WebConsoleNode.rack_snapshot(dummy)
    assert snapshot["scene_frame"] == "odom"
    assert snapshot["scene_racks"][0]["layers"][0]["boundary"][0][0] == pytest.approx(2.6)


def test_semantic_scene_uses_slam_map_frame_without_nvblox():
    calls = []

    def lookup(target, source, _stamp):
        calls.append((target, source))
        return transform(x=3.0)

    dummy = SimpleNamespace(
        lock=threading.RLock(), rack_model=MODEL, last_rack_time=time.monotonic(),
        map_frame="map", mesh_frame="",
        tf_buffer=SimpleNamespace(lookup_transform=lookup),
    )
    snapshot = WebConsoleNode.rack_snapshot(dummy)
    assert snapshot["scene_frame"] == "map"
    assert snapshot["scene_racks"] == snapshot["map_racks"]
    assert calls == [("map", "sim_world")]


def test_page_has_rack_overlay_and_list():
    web = Path(__file__).parents[1] / "web"
    page = (web / "index.html").read_text()
    script = (web / "app.js").read_text()
    assert 'id="rackSourceBadge"' in page
    assert 'id="rackList"' in page
    assert 'api("/api/racks")' in script
    assert "rackView.map_racks" in script
    assert "rackView.scene_racks" in script
    assert "outline.boundary.forEach" in script


def test_semantic_scene_is_default_and_nvblox_is_optional():
    web = Path(__file__).parents[1] / "web"
    page = (web / "index.html").read_text()
    script = (web / "app.js").read_text()
    renderer = (web / "rack_scene.js").read_text()
    assert 'id="showScene3d"' in page
    assert 'id="showNvblox3d"' in page
    assert 'src="/rack_scene.js?v=2"' in page
    assert 'showSpatialView("semantic")' in script
    assert 'drawSemanticRacks(ctx,project,racks,sceneZoom,sceneTopDown)' in script
    assert 'if(sceneMode!=="nvblox")return' in script
    assert 'layer.boundary' in renderer
    assert 'layer.observed' in renderer


def test_spatial_view_has_world_reference_and_robot_follow_controls():
    web = Path(__file__).parents[1] / "web"
    page = (web / "index.html").read_text()
    script = (web / "app.js").read_text()
    renderer = (web / "rack_scene.js").read_text()
    for element in ("sceneReference", "sceneTopDown", "sceneFollowRobot", "resetView"):
        assert f'id="{element}"' in page
    assert "drawWorldGrid(ctx,project,w,h)" in script
    assert "drawSceneReference(ctx,project,w,h)" in script
    assert "sceneTopDown)return(x,y,z)" in script
    assert "sceneFollowRobot=false;sceneAutoFit=false" in script
    assert "topDown ? [valid.reduce" in renderer
    assert "metricTickStep(sceneZoom)" in script
    assert "metricDistanceLabel(step)" in script
    assert "drawMetricRulers(ctx,project,w,h,step)" in script
    assert 'X 坐标 / cm' in script
    assert 'Y 坐标 / cm' in script
    assert 'Z 高度 / cm' in script
    assert 'const label=`${axis} ${metricCoordinateLabel(value)}`' in script
    assert 'const anchor={x:w-64,y:h-78}' not in script
    assert 'src="/app.js?v=22"' in page


def test_slam_map_uses_matching_frame_and_centimeter_cell_details():
    web = Path(__file__).parents[1] / "web"
    page = (web / "index.html").read_text()
    script = (web / "app.js").read_text()
    for element in ("mapReference", "mapProbe"):
        assert f'id="{element}"' in page
    assert 'mapPose=latestStatus?.map_pose?.online' in script
    assert 'if(trailFrame!==frame)' in script
    assert 'rackView?.map_frame===frame' in script
    assert 'drawOccupancyCellGrid(ctx,cx,cy,ppm,w,h)' in script
    assert 'occupancyAtWorld(point.x,point.y)' in script
    assert 'mapSpan=Math.max(.625,mapSpan/2)' in script
    assert '格距 ${metricDistanceLabel(step)}' in script
    assert '网格 ${metricDistanceLabel(grid)}' in script
    assert 'src="/app.js?v=22"' in page
    assert 'href="/visualization.css?v=11"' in page


def test_page_has_live_rgb_and_instance_mask():
    web = Path(__file__).parents[1] / "web"
    page = (web / "index.html").read_text()
    script = (web / "app.js").read_text()
    assert 'id="cameraRgb"' in page
    assert 'id="cameraMask"' in page
    assert '/api/camera/${kind}.jpg' in script


def test_camera_snapshot_expires_stale_images():
    dummy = SimpleNamespace(
        lock=threading.RLock(), camera_jpegs={"rgb": b"jpeg"},
        camera_times={"rgb": time.monotonic()},
    )
    assert WebConsoleNode.camera_snapshot(dummy, "rgb") == b"jpeg"
    dummy.camera_times["rgb"] -= 4.0
    assert WebConsoleNode.camera_snapshot(dummy, "rgb") is None
    assert WebConsoleNode.camera_snapshot(dummy, "mask") is None
