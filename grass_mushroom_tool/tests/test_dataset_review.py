"""离线查看器：筛选、数据验证及毫米值无损导出。"""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

spec = importlib.util.spec_from_file_location('review_dataset', Path(__file__).parents[1]/'review_dataset.py')
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)


def frame(root, number=1, aligned=True, **changes):
    p = root/'session_1'/'samples'/'S001'
    p.mkdir(parents=True, exist_ok=True)
    suffix = '' if number == 1 else f'-{number}'
    Image.new('RGB', (3, 2)).save(p/f's001-d435i-color{suffix}.png')
    Image.fromarray(np.array([[0, 1, 255], [256, 1234, 65535]], dtype=np.uint16)).save(p/f's001-d435i-depth{suffix}.png')
    color = dict(width=3, height=2, frame_id='color_optical_frame', stamp=dict(sec=100, nanosec=0))
    depth = dict(color, unit='mm', scale_mm=1)
    depth.update(changes)
    meta = dict(alignment=dict(aligned_to_color=aligned, dt_ms=99), color=color, depth=depth)
    (p/f's001-d435i-meta{suffix}.json').write_text(json.dumps(meta))
    return p


def test_report_ignores_raw_preserves_integer_depth_and_recomputes_dt(tmp_path):
    source, output = tmp_path/'data', tmp_path/'report'
    p = frame(source)
    frame(source, 2, aligned=False)
    before = {f.name: f.read_bytes() for f in p.iterdir()}
    result = review.build_report(source, output)
    assert result['aligned_frames'] == 1
    assert len(result['skipped']) == 1
    packed = np.array(Image.open(next((output/'assets').glob('*-depth.png')))).astype(np.uint16)
    assert (packed[:,:,0]*256 + packed[:,:,1]).tolist() == [[0, 1, 255], [256, 1234, 65535]]
    assert ',0.0,' in (output/'summary.csv').read_text(encoding='utf-8-sig')
    assert before == {f.name: f.read_bytes() for f in p.iterdir()}
    page = (output/'index.html').read_text()
    assert '__DATA__' not in page and '__SUMMARY__' not in page
    assert 'data:image/png;base64,' in page
    assert 'data:image/jpeg;base64,' in page


@pytest.mark.parametrize('changes', [dict(width=4), dict(unit='m'), dict(frame_id='depth_optical_frame')])
def test_invalid_metadata_skipped_with_reason(tmp_path, changes):
    frame(tmp_path/'data', **changes)
    result = review.build_report(tmp_path/'data', tmp_path/'report')
    assert result['aligned_frames'] == 0
    assert result['skipped'][0]['reason']


def test_source_levels_and_sample_filter(tmp_path):
    source = tmp_path/'data'
    sample = frame(source)
    for i, path in enumerate([source, sample.parent.parent, sample.parent, sample]):
        result = review.build_report(path, tmp_path/f'report{i}', 'S001')
        assert result['aligned_frames'] == 1


def test_broken_group_does_not_hide_valid_group(tmp_path):
    source = tmp_path/'data'
    sample = frame(source)
    frame(source, 2)
    (sample/'s001-d435i-depth-2.png').unlink()
    result = review.build_report(source, tmp_path/'report')
    assert result['aligned_frames'] == 1
    assert len(result['skipped']) == 1


def test_no_output_written_into_original_dataset(tmp_path):
    source = tmp_path/'data'
    frame(source)
    with pytest.raises(ValueError):
        review.build_report(source, source/'report')
