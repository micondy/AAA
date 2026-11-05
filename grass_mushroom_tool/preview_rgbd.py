#!/usr/bin/env python3
"""将原始 RGB-D 录制转为可直接观看的并排预览 MP4，不保存单帧照片。"""
import argparse
from pathlib import Path
from gmkit.playback import make_preview

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--camera-prefix',default='/sensors/nav_camera')
    args=p.parse_args()
    result=make_preview(args.bag,args.output,args.camera_prefix)
    print(f"已生成 {result['duration_seconds']:.2f} 秒预览：{args.output}")
