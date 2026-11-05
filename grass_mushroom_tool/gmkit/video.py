"""无损 RGB-D 连续录制的离线同步抽帧与网页后台任务。"""
from __future__ import annotations

import collections
import json
import math
import threading
import time
from pathlib import Path

from . import naming
from .rosbridge import RosBridge, RosError, _stamp_ns
from .store import Session


def video_topics(prefix):
    prefix = prefix.rstrip('/')
    return [f'{prefix}/color/image_raw', f'{prefix}/aligned_depth_to_color/image_raw',
            f'{prefix}/color/camera_info', f'{prefix}/aligned_depth_to_color/camera_info']


class ExactPairs:
    """按原始 header 纳秒配对；有界缓存，只返回完全相同时间戳的组。"""
    def __init__(self, capacity=120):
        self.pending = {kind: collections.OrderedDict() for kind in ('color', 'depth')}
        self.capacity = capacity
        self.dropped = 0
        self.bad_stamp = 0
        self.seen = collections.OrderedDict()

    def push(self, kind, msg):
        stamp = _stamp_ns(msg)
        if stamp is None:
            self.bad_stamp += 1
            return None
        if stamp in self.seen:
            return None
        self.pending[kind][stamp] = msg
        if all(stamp in queue for queue in self.pending.values()):
            color, depth = (self.pending[key].pop(stamp) for key in ('color', 'depth'))
            self.seen[stamp] = True
            if len(self.seen) > self.capacity * 2:
                self.seen.popitem(last=False)
            return color, depth
        if len(self.pending[kind]) > self.capacity:
            self.pending[kind].popitem(last=False)
            self.dropped += 1
        return None

    @property
    def unmatched(self):
        return self.dropped + sum(len(queue) for queue in self.pending.values())


def extract_bag(bag_path, output, *, prefix='/sensors/nav_camera', sample='S001', interval=0.5,
                stride=2, progress=None):
    """读取 MCAP/SQLite rosbag，严格同步抽帧；不按 rosbag 接收时间凑帧。"""
    if not math.isfinite(float(interval)) or interval < 0:
        raise RosError('抽帧间隔必须是有限的非负秒数')
    if int(stride) < 1:
        raise RosError('点云抽样步长必须大于 0')
    bag_path, output = Path(bag_path).resolve(), Path(output).resolve()
    if not (bag_path / 'metadata.yaml').is_file():
        raise RosError('录制尚未正常结束或缺少 metadata.yaml，请先停止录制')
    if output.exists():
        raise RosError('抽帧输出目录已存在，请选择新目录以避免覆盖')
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image, CameraInfo
    except ImportError as exc:
        raise RosError('抽帧需要 ROS 2 Python 环境，请先 source /opt/ros/jazzy/setup.bash') from exc
    topics = video_topics(prefix)

    def reader_for(selected):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag_path)), rosbag2_py.ConverterOptions('', ''))
        available = {topic.name for topic in reader.get_all_topics_and_types()}
        if not set(topics[:3]).issubset(available):
            raise RosError('录制缺少 RGB、对齐深度或彩色相机内参，无法同步抽帧')
        reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
        return reader

    # 内参通常稳定；先读首条作为兜底，处理 bag 中 camera_info 晚于图像到达的情况。
    reader = reader_for(topics[2:])
    initial_info = {}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        initial_info[topic] = deserialize_message(raw, CameraInfo)
        if topics[2] in initial_info:
            break
    del reader
    if topics[2] not in initial_info:
        raise RosError('录制中没有彩色相机内参消息')
    reader = reader_for(topics)
    sample = naming.normalize_sample_id(sample)
    output.mkdir(parents=True, exist_ok=False)
    session = Session(output)
    session.config = {'id': output.name, 'date': time.strftime('%Y%m%d'), 'operator': '', 'rooms': [],
                      'notes': f'RGB-D 抽帧，来源 {bag_path}', 'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                      'next_index': int(sample[1:])}
    entry = session.add_sample(notes='RGB-D 录制抽帧；尺寸和重量尚未人工测量')
    bridge, pairs = RosBridge(prefix), ExactPairs()
    infos = initial_info
    stats = {'bag': str(bag_path), 'output': str(output), 'sample': sample, 'interval_seconds': interval,
             'messages': 0, 'matched_pairs': 0, 'saved': 0, 'skipped_interval': 0, 'invalid_pairs': 0,
             'unmatched_messages': 0, 'invalid_stamps': 0, 'errors': [], 'complete': False}
    last_saved = None
    try:
        while reader.has_next():
            topic, raw, bag_time = reader.read_next()
            stats['messages'] += 1
            if topic in topics[2:]:
                infos[topic] = deserialize_message(raw, CameraInfo)
                continue
            msg = deserialize_message(raw, Image)
            pair = pairs.push('color' if topic == topics[0] else 'depth', msg)
            if pair is None:
                continue
            stats['matched_pairs'] += 1
            stamp = _stamp_ns(pair[0])
            if last_saved is not None and stamp - last_saved < round(interval*1e9):
                stats['skipped_interval'] += 1
                continue
            try:
                files = bridge.save_pair(output, sample, *pair, infos.get(topics[2]), infos.get(topics[3]),
                    seq=stats['saved'] + 1, stride=stride,
                    source={'type': 'rosbag2', 'bag': str(bag_path), 'bag_receive_time_ns': int(bag_time),
                            'camera_stamp_ns': stamp})
            except RosError as exc:
                stats['invalid_pairs'] += 1
                if len(stats['errors']) < 20:
                    stats['errors'].append(str(exc))
                continue
            for item in files:
                path = output/item['rel']
                entry['photos'].append(session._photo_entry(path.name, item['rel'], f"d435i-{item['kind']}", item['bytes'], 'rgbd-video'))
            stats['saved'] += 1
            last_saved = stamp
            if progress:
                progress(dict(stats))
        stats['complete'] = True
    finally:
        stats['unmatched_messages'] = pairs.unmatched
        stats['invalid_stamps'] = pairs.bad_stamp
        session.save()
        (output/'extraction.json').write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding='utf-8')
        del reader
    if not stats['saved']:
        raise RosError('没有可导出的同步帧：请检查相机 enable_sync、对齐深度和内参；详见 extraction.json')
    return stats


class ExtractionJob:
    """单个后台抽帧任务，供网页查询进度，避免 HTTP 请求长时间阻塞。"""
    def __init__(self):
        self._lock = threading.Lock()
        self._state = {'running': False}
        self._thread = None

    def status(self):
        with self._lock:
            return dict(self._state)

    def start(self, bag, output, **options):
        with self._lock:
            if self._state['running']:
                raise RosError('已有抽帧任务正在运行')
            self._state = {'running': True, 'saved': 0, 'bag': str(bag), 'output': str(output), 'error': None}
        def update(stats):
            with self._lock:
                self._state.update(stats)
        def run():
            try:
                update(extract_bag(bag, output, progress=update, **options))
            except Exception as exc:
                update({'error': str(exc)})
            finally:
                update({'running': False})
        self._thread = threading.Thread(target=run, name='rgbd-extract', daemon=False)
        self._thread.start()
        return self.status()
