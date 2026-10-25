from types import SimpleNamespace

import numpy as np
import pytest

from rack_perception.yolo_masks import colorize_mask, encode_yolo_result, validate_rack_classes


def result(classes, scores, masks):
    return SimpleNamespace(
        boxes=SimpleNamespace(cls=np.asarray(classes), conf=np.asarray(scores)),
        masks=SimpleNamespace(data=np.asarray(masks)),
    )


def test_rack_weights_require_custom_bed_post_classes():
    validate_rack_classes({0: "bed", 1: "post"})
    with pytest.raises(ValueError):
        validate_rack_classes({0: "person", 1: "bicycle"})


def test_yolo_masks_use_frame_local_bed_and_post_ranges_and_confidence_order():
    masks = np.zeros((3, 8, 8), dtype=np.uint8)
    masks[0, 1:5, 1:5] = 1
    masks[1, 3:7, 3:7] = 1
    masks[2, 0:3, 6:8] = 1
    output, items = encode_yolo_result(result([0, 0, 1], [0.9, 0.7, 0.8], masks), (8, 8), 1)
    assert [item["mask_id"] for item in items] == [1, 10000, 2]
    assert output[3, 3] == 1  # overlapping pixel belongs to the better bed
    assert output[6, 6] == 2
    assert output[1, 7] == 10000
    assert colorize_mask(output).shape == (8, 8, 3)


def test_empty_and_resized_results():
    empty, items = encode_yolo_result(SimpleNamespace(boxes=None, masks=None), (10, 12))
    assert not items and not empty.any()
    output, items = encode_yolo_result(
        result([0], [0.95], np.ones((1, 2, 3), dtype=np.uint8)), (10, 12), 5
    )
    assert output.shape == (10, 12)
    assert items[0]["pixels"] == 120
