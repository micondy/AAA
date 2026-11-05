#!/usr/bin/env python3
"""从 RGB-D 连续录制中抽取严格同步的 RGB、16 位深度和同帧点云。"""
import argparse
from pathlib import Path
from gmkit.video import extract_bag
from gmkit.rosbridge import RosError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True, help='已正常停止的 rosbag 目录')
    parser.add_argument('--output', type=Path, required=True, help='新的输出会话目录，不能已存在')
    parser.add_argument('--sample', default='S001')
    parser.add_argument('--camera-prefix', default='/sensors/nav_camera')
    parser.add_argument('--interval', type=float, default=0.5, help='抽帧间隔秒数；0 表示导出全部同步帧')
    parser.add_argument('--stride', type=int, default=2, help='点云在两个像素方向的抽样步长')
    args = parser.parse_args()
    try:
        result = extract_bag(args.bag, args.output, sample=args.sample, prefix=args.camera_prefix,
                             interval=args.interval, stride=args.stride)
    except (RosError, OSError, ValueError) as exc:
        parser.exit(1, f'抽帧失败：{exc}\n')
    print(f"同步候选 {result['matched_pairs']} 组，保存 {result['saved']} 组，未配对消息 {result['unmatched_messages']} 条")
    print(f'输出：{args.output.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
