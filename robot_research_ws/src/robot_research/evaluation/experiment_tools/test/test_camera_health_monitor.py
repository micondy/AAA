from sensor_msgs.msg import Image

from experiment_tools.camera_health_monitor import StreamState, stamp_seconds
from experiment_tools.camera_topics import camera_prefix, camera_topics, depth_topic


def test_stamp_seconds_combines_ros_timestamp():
    message = Image()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345_000_000

    assert stamp_seconds(message) == 12.345


def test_stream_state_defaults_are_safe():
    state = StreamState("color", 20.0)

    assert state.count == 0
    assert state.previous_count == 0
    assert state.last_stamp is None


def test_basic_stage_records_raw_depth_without_imu():
    topics = camera_topics("camera", "camera", "basic")

    assert "/camera/camera/depth/image_rect_raw" in topics
    assert "/camera/camera/aligned_depth_to_color/image_raw" not in topics
    assert "/camera/camera/imu" not in topics


def test_full_stage_records_aligned_depth_and_imu():
    topics = camera_topics("robot1/sensors", "front", "full")

    assert "/robot1/sensors/front/aligned_depth_to_color/image_raw" in topics
    assert "/robot1/sensors/front/imu" in topics
    assert depth_topic("robot1/sensors", "front", "full").endswith(
        "/aligned_depth_to_color/image_raw"
    )


def test_camera_prefix_normalizes_slashes():
    assert camera_prefix("/robot1/sensors/", "front") == "/robot1/sensors/front"
