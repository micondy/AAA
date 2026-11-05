#!/usr/bin/env python3
"""读取草菇 RGB-D 数据，生成仅含已对齐帧的离线检查工具（不修改原始数据）。"""
from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
import re
import sys
import webbrowser
from pathlib import Path
from urllib.parse import quote
import os

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent
META_NAME = re.compile(r"(.+)-d435i-meta(-([1-9][0-9]*))?\.json$")


def find_samples(source: Path):
    """仅检查约定目录，不递归扫描点云、其他数据或工作区。"""
    if source.name == 'samples':
        return [(source.parent.name, p) for p in sorted(source.iterdir()) if p.is_dir()]
    if (source / 'samples').is_dir():
        return [(source.name, p) for p in sorted((source / 'samples').iterdir()) if p.is_dir()]
    if next(source.glob('*-d435i-meta*.json'), None):
        session = source.parent.parent.name if source.parent.name == 'samples' else source.parent.name
        return [(session, source)]
    found = []
    for session in sorted(source.iterdir()):
        if session.is_dir() and (session / 'samples').is_dir():
            found.extend(find_samples(session))
    return found


def relative_url(path: Path, output: Path):
    return quote(Path(os.path.relpath(path, output)).as_posix(), safe='/')


def build_report(source: Path, output: Path, sample_filter=None):
    source, output = source.expanduser().resolve(), output.expanduser().resolve()
    if not source.is_dir():
        raise ValueError(f'数据目录不存在：{source}')
    if source == output or source in output.parents:
        raise ValueError('报告输出目录应放在原始数据目录之外')
    candidates = find_samples(source)
    if sample_filter:
        candidates = [(session, p) for session, p in candidates if p.name.upper() == sample_filter.upper()]
    if not candidates:
        raise ValueError('未找到样本，请指定 data/、会话目录或 samples/S002 目录')
    output.mkdir(parents=True, exist_ok=True)
    assets = output / 'assets'
    assets.mkdir(exist_ok=True)
    frames, skipped = [], []
    for session, sample in candidates:
        paths = sorted(sample.glob('*-d435i-meta*.json'), key=lambda p: int((META_NAME.fullmatch(p.name).group(3) or 1)) if META_NAME.fullmatch(p.name) else 0)
        for meta_path in paths:
            match = META_NAME.fullmatch(meta_path.name)
            if not match:
                continue
            prefix, suffix, number = match.groups()
            suffix = suffix or ''
            group = int(number or 1)
            label = f'{session}/{sample.name}/{group}'
            try:
                meta = json.loads(meta_path.read_text(encoding='utf-8'))
                if meta.get('alignment', {}).get('aligned_to_color') is not True:
                    skipped.append({'frame': label, 'reason': '未对齐或缺少对齐标记'})
                    continue
                color_path = sample / f'{prefix}-d435i-color{suffix}.png'
                depth_path = sample / f'{prefix}-d435i-depth{suffix}.png'
                with Image.open(color_path) as rgb_source:
                    rgb = rgb_source.convert('RGB')
                with Image.open(depth_path) as dep:
                    depth = np.asarray(dep)
                    if dep.format != 'PNG' or dep.mode not in ('I', 'I;16', 'I;16B', 'I;16L'):
                        raise ValueError('深度不是 16 位整数 PNG')
                if depth.ndim != 2 or depth.shape != (rgb.height, rgb.width):
                    raise ValueError('RGB 与深度尺寸不一致')
                if depth.min() < 0 or depth.max() > 65535:
                    raise ValueError('深度数值超出 uint16 范围')
                color_meta, depth_meta = meta.get('color', {}), meta.get('depth', {})
                if depth_meta.get('unit') != 'mm' or depth_meta.get('scale_mm') != 1:
                    raise ValueError('未确认深度单位为 1 mm，不能显示毫米读数')
                if not color_meta.get('frame_id') or color_meta['frame_id'] != depth_meta.get('frame_id'):
                    raise ValueError('光学坐标系不一致或缺失')
                for record in (color_meta, depth_meta):
                    if (record.get('width'), record.get('height')) != rgb.size:
                        raise ValueError('元数据尺寸与图像不一致')
                depth = depth.astype(np.uint16)
                valid = depth[depth > 0]
                lo, mid, hi = (int(v) for v in np.percentile(valid, [2, 50, 98])) if valid.size else (0, 0, 0)
                dt = meta.get('alignment', {}).get('dt_ms')
                stamps = [record.get('stamp', {}) for record in (color_meta, depth_meta)]
                if all('sec' in stamp and 'nanosec' in stamp for stamp in stamps):
                    ns = [int(stamp['sec'])*10**9 + int(stamp['nanosec']) for stamp in stamps]
                    dt = round(abs(ns[0]-ns[1])/1e6, 3)
                elif dt is not None:
                    dt = float(dt)
                key = hashlib.sha256(str(meta_path).encode()).hexdigest()[:20]
                rgb.save(assets / f'{key}-rgb.jpg', quality=92)
                packed = np.zeros((*depth.shape, 3), dtype=np.uint8)
                packed[:, :, 0] = depth >> 8
                packed[:, :, 1] = depth & 255
                Image.fromarray(packed).save(assets / f'{key}-depth.png')
                frames.append(dict(session=session, sample=sample.name, group=group, dt=dt,
                    time=meta.get('captured_at', ''), valid=round(float((depth > 0).mean()*100), 2),
                    lo=lo, hi=hi, median=mid, width=rgb.width, height=rgb.height,
                    rgb='data:image/jpeg;base64,' + base64.b64encode((assets/f'{key}-rgb.jpg').read_bytes()).decode('ascii'),
                    depth='data:image/png;base64,' + base64.b64encode((assets/f'{key}-depth.png').read_bytes()).decode('ascii'),
                    originals={kind: relative_url(path, output) for kind, path in
                        [('color', color_path), ('depth', depth_path), ('meta', meta_path)]}))
            except (OSError, ValueError, TypeError, KeyError) as exc:
                skipped.append({'frame': label, 'reason': str(exc)})
    template = (ROOT / 'web/dataset_review.html').read_text(encoding='utf-8')
    # JSON 写入 script 时禁止提前闭合标签。
    data = json.dumps(frames, ensure_ascii=False).replace('<', '\\u003c')
    summary = {'source': str(source), 'aligned_frames': len(frames), 'skipped': skipped}
    summary_json = json.dumps(summary, ensure_ascii=False).replace('<', '\\u003c')
    (output / 'index.html').write_text(template.replace('__DATA__', data).replace('__SUMMARY__', summary_json), encoding='utf-8')
    (output / 'inspection.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    fields = ['session', 'sample', 'group', 'time', 'dt', 'valid', 'lo', 'median', 'hi']
    with (output / 'summary.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(frames)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=ROOT/'data', help='数据根目录、会话目录或单个样本目录')
    parser.add_argument('--sample', help='仅导出指定样本，例如 S002')
    parser.add_argument('--output', type=Path, default=ROOT/'reports/dataset_review')
    parser.add_argument('--open', action='store_true', help='生成后用默认浏览器打开')
    args = parser.parse_args()
    try:
        result = build_report(args.data, args.output, args.sample)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'错误：{exc}\n')
    path = args.output.expanduser().resolve()/'index.html'
    print(f"已对齐可查看：{result['aligned_frames']} 组；跳过：{len(result['skipped'])} 组")
    print(f'查看页：{path}')
    print('跳过原因：同目录 inspection.json；汇总：summary.csv')
    if args.open:
        webbrowser.open(path.as_uri())
    return 0


if __name__ == '__main__':
    sys.exit(main())
