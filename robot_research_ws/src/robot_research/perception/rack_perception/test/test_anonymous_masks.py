import random

from rack_perception.detector_node import anonymous_mask_ids


def test_anonymous_instance_ids_are_unique_and_change_between_frames():
    visible = [
        (101, 11, "rack_01_layer_1", 1000),
        (102, 12, "rack_01_layer_2", 900),
        (201, 21, "rack_02_layer_1", 800),
        (301, 10001, "rack_01_post", 200),
    ]
    rng = random.Random(2026)
    first = anonymous_mask_ids(visible, rng)
    second = anonymous_mask_ids(visible, rng)
    assert first != second
    assert len(set(first.values())) == len(visible)
    assert len(set(second.values())) == len(visible)
    for mapping in (first, second):
        assert all(0 < mapping[source] < 10000 for source in (101, 102, 201))
        assert 10000 <= mapping[301] <= 65535
