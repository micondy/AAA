import pytest

from target_perception.instance_tracking import (
    ClassAware3DTracker,
    InstanceObservation,
)


def _observation(class_id, x, y=0.0, z=1.0):
    return InstanceObservation(class_id, (x, y, z))


def test_nearby_same_class_keeps_id_and_far_instance_gets_new_id():
    tracker = ClassAware3DTracker(max_distance_m=0.3)
    first = tracker.update([_observation("cup", 0.0)])
    second = tracker.update([_observation("cup", 0.1), _observation("cup", 1.0)])

    assert first == ["cup_0001"]
    assert second == ["cup_0001", "cup_0002"]
    assert tracker.tracks["cup_0001"].hits == 2


def test_classes_never_associate_even_at_same_position():
    tracker = ClassAware3DTracker(max_distance_m=1.0)
    assert tracker.update([_observation("cup", 0.0)]) == ["cup_0001"]
    assert tracker.update([_observation("bottle", 0.0)]) == ["bottle_0001"]


def test_global_greedy_assignment_does_not_reuse_a_track():
    tracker = ClassAware3DTracker(max_distance_m=0.5)
    tracker.update([_observation("person", 0.0), _observation("person", 0.4)])
    ids = tracker.update([_observation("person", 0.1), _observation("person", 0.3)])
    assert set(ids) == {"person_0001", "person_0002"}


def test_track_expires_after_configured_missed_frames():
    tracker = ClassAware3DTracker(max_distance_m=0.3, max_missed_frames=1)
    tracker.update([_observation("book", 0.0)])
    tracker.update([])
    assert "book_0001" in tracker.tracks
    tracker.update([])
    assert "book_0001" not in tracker.tracks
    assert tracker.update([_observation("book", 0.0)]) == ["book_0002"]


def test_reappearing_observation_cannot_revive_an_expired_track():
    tracker = ClassAware3DTracker(max_distance_m=0.3, max_missed_frames=1)
    tracker.update([_observation("chair", 0.0)])
    tracker.update([])
    tracker.update([])
    assert tracker.update([_observation("chair", 0.0)]) == ["chair_0002"]


def test_invalid_observation_is_rejected():
    tracker = ClassAware3DTracker()
    with pytest.raises(ValueError, match="finite"):
        tracker.update([_observation("cup", float("nan"))])
