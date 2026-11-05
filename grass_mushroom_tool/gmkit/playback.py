"""将原始 RGB-D 录制流式转为可播放预览，不导出照片或改动原始数据。"""
from pathlib import Path
import json
import math
import threading

from . import media
from .rosbridge import RosError, _image_array, _stamp_ns
from .video import ExactPairs, video_topics


def make_preview(bag, output, prefix='/sensors/nav_camera', near=150, far=1000, progress=None):
    import cv2
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    bag, output = Path(bag).resolve(), Path(output).resolve()
    if not (bag/'metadata.yaml').is_file():
        raise RosError('请先正常停止录制')
    if not math.isfinite(near) or not math.isfinite(far) or not 0 < near < far:
        raise RosError('深度显示范围不合法')
    if output.exists():
        raise RosError('预览已存在，请直接播放；不覆盖已有文件')
    topics = video_topics(prefix)[:2]
    def open_reader(selected):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag)), rosbag2_py.ConverterOptions('', ''))
        available = {t.name for t in reader.get_all_topics_and_types()}
        if not set(topics).issubset(available):
            raise RosError('原始录制缺少 RGB 或对齐深度')
        reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
        return reader
    reader = open_reader(topics[:1])
    stamps = []
    while reader.has_next():
        _, raw, _ = reader.read_next()
        stamp = _stamp_ns(deserialize_message(raw, Image))
        if stamp is not None:
            stamps.append(stamp)
    del reader
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        raise RosError('录制中有效彩色帧不足，不能生成视频')
    fps = (len(stamps)-1) * 1e9 / (stamps[-1]-stamps[0])
    if not 1 <= fps <= 240:
        raise RosError('相机时间戳异常，无法确定播放帧率')
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.stem+'.partial.mp4')
    reader, pairs = open_reader(topics), ExactPairs()
    writer = None
    count = matched = repeated = 0
    first_stamp = None
    last_frame = None
    size = None
    try:
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            pair = pairs.push('color' if topic == topics[0] else 'depth', deserialize_message(raw, Image))
            if pair is None:
                continue
            color, depth = pair
            if color.header.frame_id != depth.header.frame_id or (color.width,color.height)!=(depth.width,depth.height):
                raise RosError('RGB 与深度尺寸或坐标系不一致')
            if depth.encoding.lower() not in ('16uc1','mono16'):
                raise RosError('深度不是 16 位毫米数据')
            rgb, dep = _image_array(color), _image_array(depth)
            false_color = media.colorize_depth(dep, near, far)
            image = np.ascontiguousarray(np.concatenate((rgb, false_color), axis=1)[:,:,::-1])
            # H.264 使用偶数宽高；仅对预览补边，不改原始数据。
            image = cv2.copyMakeBorder(image,0,image.shape[0]%2,0,image.shape[1]%2,cv2.BORDER_CONSTANT)
            current_size = (image.shape[1], image.shape[0])
            if writer is None:
                size = current_size
                writer = cv2.VideoWriter(str(temporary),cv2.VideoWriter_fourcc(*'avc1'),fps,size)
                if not writer.isOpened():
                    raise RosError('当前 OpenCV 缺少 H.264 编码器')
            if current_size != size:
                raise RosError('录制中分辨率发生变化，无法合并为一个预览视频')
            stamp = _stamp_ns(color)
            if first_stamp is None:
                first_stamp = stamp
            target = round((stamp-first_stamp)*fps/1e9)
            if target < count:
                continue
            while count < target and last_frame is not None:
                writer.write(last_frame)
                count += 1
                repeated += 1
            cv2.putText(image,'RGB',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
            cv2.putText(image,f'Depth {near:g}-{far:g} mm | black=invalid',(rgb.shape[1]+12,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
            writer.write(image)
            last_frame = image
            count += 1
            matched += 1
            if progress and matched%30==0:
                progress({'frames':matched})
        if not matched:
            raise RosError('未找到同一时间戳的 RGB-D 帧，无法生成同步预览')
        writer.release()
        writer = None
        # 不覆盖并发生成的已有预览。
        with output.open('xb') as target_file, temporary.open('rb') as source:
            import shutil
            shutil.copyfileobj(source,target_file)
        result = {'source':str(bag),'output':str(output),'frames':matched,'encoded_frames':count,
                  'fps':fps,'duration_seconds':count/fps,'repeated_frames':repeated,
                  'unmatched_messages':pairs.unmatched,'near_mm':near,'far_mm':far,
                  'note':'RGB + depth false color preview; quantitative depth remains in the original MCAP'}
        output.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        return result
    finally:
        if writer is not None:
            writer.release()
        temporary.unlink(missing_ok=True)
        del reader


class PreviewJob:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = {'running':False}

    def status(self):
        with self.lock:
            return dict(self.state)

    def start(self, bag, output, prefix):
        with self.lock:
            if self.state['running']:
                raise RosError('已有预览视频正在生成')
            self.state = {'running':True,'frames':0,'output':str(output),'error':None}
        def update(values):
            with self.lock:
                self.state.update(values)
        def run():
            try:
                update(make_preview(bag,output,prefix,progress=update))
            except Exception as exc:
                update({'error':str(exc)})
            finally:
                update({'running':False})
        threading.Thread(target=run,name='rgbd-preview',daemon=False).start()
        return self.status()
