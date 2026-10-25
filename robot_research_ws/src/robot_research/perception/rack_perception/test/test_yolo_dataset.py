import numpy as np

from rack_perception.yolo_dataset import mask_to_yolo_rows


def test_bed_and_disconnected_posts_convert_to_yolo_polygons():
    mask = np.zeros((80, 100), dtype=np.uint16)
    mask[10:40, 10:70] = 12
    mask[45:65, 10:16] = 10001
    mask[45:65, 75:81] = 10001
    rows, counts = mask_to_yolo_rows(mask, min_bed_pixels=30, min_post_pixels=10)
    assert counts == {"bed": 1, "post": 2}
    assert [int(row.split()[0]) for row in rows] == [0, 1, 1]
    for row in rows:
        values = [float(value) for value in row.split()[1:]]
        assert len(values) >= 6 and len(values) % 2 == 0
        assert all(0.0 <= value <= 1.0 for value in values)


def test_background_and_tiny_fragments_are_ignored():
    mask = np.zeros((20, 20), dtype=np.uint16)
    mask[1:3, 1:3] = 1
    assert mask_to_yolo_rows(mask)[1] == {"bed": 0, "post": 0}
