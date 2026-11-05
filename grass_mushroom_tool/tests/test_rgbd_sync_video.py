import json
import numpy as np
import pytest
from gmkit import naming
from gmkit.rosbridge import RosBridge, RosError
from gmkit.video import ExactPairs, extract_bag
from test_rosbridge import rgb8_image, depth16_image, Info


def test_pair_uses_latest_complete_cached_frame():
    bridge = RosBridge('/cam')
    bridge._keep('color', rgb8_image(sec=100))
    bridge._keep('color', rgb8_image(sec=101))
    bridge._keep_depth(depth16_image(sec=100), True)
    color, depth, delta = bridge._pair(timeout=0)
    assert color[0].header.stamp.sec == depth[0].header.stamp.sec == 100
    assert delta == 0


def test_nanosecond_mismatch_at_epoch_scale_is_rejected():
    bridge = RosBridge('/cam')
    color, depth = rgb8_image(sec=1793873496), depth16_image(sec=1793873496)
    color.header.stamp.nanosec = 541972656
    depth.header.stamp.nanosec = 541972657
    bridge._keep_depth(depth, True)
    picked, dt = bridge._pick_depth(color)
    assert picked is None
    assert dt == pytest.approx(1e-9)


def test_missing_stamp_is_not_paired_by_arrival():
    bridge = RosBridge('/cam')
    bridge._keep('color', rgb8_image(sec=0))
    bridge._keep_depth(depth16_image(sec=0), True)
    assert bridge._pair(timeout=0)[1] is None


def test_generated_points_geometry_color_timestamp_and_no_overwrite(tmp_path):
    bridge = RosBridge('/cam')
    color, depth, info = rgb8_image(sec=100), depth16_image(sec=100), Info(3, 2)
    info.k = [100.,0.,1.,0.,100.,.5,0.,0.,1.]
    bridge.save_pair(tmp_path, 'S002', color, depth, info, stride=1)
    folder = tmp_path/'samples/S002'
    meta = json.loads((folder/'s002-d435i-meta.json').read_text())
    assert meta['pointcloud']['frame_id'] == color.header.frame_id
    assert meta['pointcloud']['stamp'] == meta['color']['stamp'] == meta['depth']['stamp']
    assert meta['pointcloud']['dt_ms'] == 0
    ply = (folder/'s002-d435i-points.ply').read_bytes().split(b'end_header\n')[1]
    points = np.frombuffer(ply, dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('r','u1'),('g','u1'),('b','u1')])
    assert points[0]['z'] == pytest.approx(.1)
    assert points[0]['x'] == pytest.approx(-.001)
    assert points[0]['y'] == pytest.approx(-.0005)
    assert [points[0][key] for key in ('r','g','b')] == [1,2,3]
    before = {p.name:p.read_bytes() for p in folder.iterdir()}
    with pytest.raises(RosError, match='拒绝覆盖'):
        bridge.save_pair(tmp_path, 'S002', color, depth, info)
    assert before == {p.name:p.read_bytes() for p in folder.iterdir()}


def test_points_require_matching_intrinsics(tmp_path):
    with pytest.raises(RosError, match='内参'):
        RosBridge('/cam').save_pair(tmp_path, 'S001', rgb8_image(), depth16_image(), None)
    assert not list(tmp_path.iterdir())


def test_sequence_uses_maximum_even_with_holes(tmp_path):
    folder = tmp_path/'samples/S002'
    folder.mkdir(parents=True)
    (folder/'s002-d435i-color.png').touch()
    (folder/'s002-d435i-depth-30.png').touch()
    assert naming.next_d435i_seq(tmp_path, 'S002') == 31


def test_offline_pairs_match_headers_with_bounded_cache():
    pairs = ExactPairs(capacity=2)
    assert pairs.push('color', rgb8_image(sec=100)) is None
    assert pairs.push('depth', depth16_image(sec=101)) is None
    pair = pairs.push('depth', depth16_image(sec=100))
    assert pair and pair[0].header.stamp.sec == pair[1].header.stamp.sec == 100
    assert pairs.push('color', rgb8_image(sec=100)) is None
    for sec in (102,103,104): pairs.push('color', rgb8_image(sec=sec))
    assert len(pairs.pending['color']) == 2 and pairs.dropped == 1


def test_real_rosbag_extract_round_trip(tmp_path):
    rosbag2 = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image, CameraInfo
    from PIL import Image as PILImage
    from gmkit.video import video_topics
    bag, output = tmp_path/'bag', tmp_path/'extracted'
    topics = video_topics('/cam')
    writer = rosbag2.SequentialWriter()
    writer.open(rosbag2.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2.ConverterOptions('', ''))
    for i, topic in enumerate(topics):
        writer.create_topic(rosbag2.TopicMetadata(id=i+1, name=topic, type='sensor_msgs/msg/'+('Image' if i<2 else 'CameraInfo'), serialization_format='cdr'))
    info = CameraInfo(width=3, height=2)
    info.header.frame_id = 'color_optical_frame'
    info.header.stamp.sec = 100
    info.k = [100.,0.,1.,0.,100.,.5,0.,0.,1.]
    def img(encoding, sec, data):
        msg = Image(width=3, height=2, encoding=encoding, step=9 if encoding=='rgb8' else 6)
        msg.header.frame_id = 'color_optical_frame'
        msg.header.stamp.sec = sec
        msg.data = data
        return msg
    rgb = img('rgb8',100,bytes(range(18)))
    values = np.array([[0,1000,1100],[1200,1300,1400]],dtype='<u2')
    depth = img('16UC1',100,values.tobytes())
    messages = [(topics[2],info),(topics[0],rgb), (topics[0],img('rgb8',101,bytes(range(18)))),
                (topics[1],depth),(topics[1],img('16UC1',102,values.tobytes()))]
    for i,(topic,msg) in enumerate(messages): writer.write(topic,serialize_message(msg),100000000000+i*1000)
    writer.close()
    result = extract_bag(bag,output,prefix='/cam',sample='S002',interval=0,stride=1)
    assert result['saved'] == result['matched_pairs'] == 1
    assert result['unmatched_messages'] == 2 and result['complete'] is True
    assert np.array_equal(np.array(PILImage.open(output/'samples/S002/s002-d435i-depth.png')),values)
    meta = json.loads((output/'samples/S002/s002-d435i-meta.json').read_text())
    assert meta['pointcloud']['dt_ms'] == 0
    assert meta['source']['camera_stamp_ns'] == 100000000000
    assert (output/'session.json').is_file()
