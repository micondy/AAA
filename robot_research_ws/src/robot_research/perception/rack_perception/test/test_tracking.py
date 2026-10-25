from dataclasses import replace

from rack_perception.model import LayerObservation
from rack_perception.tracking import RackTracker


def observation(height, x=2.0):
    return LayerObservation(
        source_id=1,
        center=(x, 0.0, height),
        axis=(0.0, 1.0),
        front_center=(x - 0.3, 0.0, height),
        front_normal=(-1.0, 0.0),
        boundary=((x - 0.3, -0.7, height), (x + 0.3, -0.7, height), (x + 0.3, 0.7, height), (x - 0.3, 0.7, height)),
        confidence=0.9,
        observed_ratio=0.8,
        point_count=1000,
    )


def test_three_layers_keep_ids_and_survive_missing_frame():
    tracker = RackTracker()
    first = [observation(value) for value in (0.42, 0.72, 1.02)]
    tracker.update(first, 1.0)
    initial = tracker.to_dict("odom", 1.0)
    ids = [item["stable_id"] for item in initial["racks"][0]["layers"]]
    tracker.update([replace(item, center=(2.02, 0.01, item.center[2] + 0.01)) for item in first], 2.0)
    updated = tracker.to_dict("odom", 2.0)
    assert [item["stable_id"] for item in updated["racks"][0]["layers"]] == ids
    assert [item["level_number"] for item in updated["racks"][0]["layers"]] == [1, 2, 3]
    tracker.update([], 3.0)
    held = tracker.to_dict("odom", 3.0)
    assert len(held["racks"]) == 1
    assert len(held["racks"][0]["layers"]) == 3
    assert not any(item["observed"] for item in held["racks"][0]["layers"])


def test_partial_rack_stays_candidate_until_three_layers_are_seen():
    tracker = RackTracker(min_confirmed_layers=3)
    tracker.update([observation(0.55)], 1.0)
    assert tracker.to_dict("sim_world", 1.0)["racks"] == []
    tracker.update([observation(0.55), observation(1.05)], 2.0)
    assert tracker.to_dict("sim_world", 2.0)["racks"] == []
    tracker.update([observation(value) for value in (0.55, 1.05, 1.55)], 3.0)
    model = tracker.to_dict("sim_world", 3.0)
    assert len(model["racks"]) == 1
    assert len(model["racks"][0]["layers"]) == 3


def test_grid_instance_hint_keeps_id_after_partial_view_shift():
    tracker = RackTracker()
    first = [replace(observation(z, 2.0), source_id=11 + i) for i, z in enumerate((0.55, 1.05, 1.55))]
    second = [replace(observation(z, 4.6), source_id=21 + i) for i, z in enumerate((0.55, 1.05, 1.55))]
    tracker.update(first + second, 1.0)
    initial = tracker.to_dict("sim_world", 1.0)
    assert len(initial["racks"]) == 2
    tracker.update([replace(first[0], center=(2.9, 0.0, 0.55))], 2.0)
    held = tracker.to_dict("sim_world", 2.0)
    assert [rack["stable_id"] for rack in held["racks"]] == [rack["stable_id"] for rack in initial["racks"]]
    assert all(len(rack["layers"]) == 3 for rack in held["racks"])
    tracker.update([replace(first[0], center=(2.0, 0.0, 0.80))], 3.0)
    rejected = tracker.to_dict("sim_world", 3.0)
    assert all(len(rack["layers"]) == 3 for rack in rejected["racks"])


def test_anonymous_masks_reassociate_two_racks_after_occlusion():
    tracker = RackTracker(
        rack_distance_m=1.8, layer_xy_distance_m=1.8,
        min_confirmed_frames=3, use_source_instance_hints=False,
    )
    heights = (0.55, 1.05, 1.55)
    for frame in range(3):
        # Frame-local mask IDs are deliberately unrelated to the rack or level.
        observations = [
            replace(observation(z, x + 0.03 * frame), source_id=100 + frame * 17 + i)
            for x in (2.0, 4.6) for i, z in enumerate(heights)
        ]
        tracker.update(list(reversed(observations)) if frame % 2 else observations, frame + 1.0)
    initial = tracker.to_dict("sim_world", 3.0)
    assert len(initial["racks"]) == 2
    assert all(len(rack["layers"]) == 3 for rack in initial["racks"])
    ids = {rack["stable_id"]: [layer["stable_id"] for layer in rack["layers"]]
           for rack in initial["racks"]}
    tracker.update([], 4.0)
    assert all(not layer["observed"] for rack in tracker.to_dict("sim_world", 4.0)["racks"]
               for layer in rack["layers"])
    returned = [replace(observation(z, x + 0.04), source_id=900 - i)
                for x in (2.0, 4.6) for i, z in enumerate(heights)]
    tracker.update(returned, 5.0)
    result = tracker.to_dict("sim_world", 5.0)
    assert len(result["racks"]) == 2
    assert {rack["stable_id"]: [layer["stable_id"] for layer in rack["layers"]]
            for rack in result["racks"]} == ids
    assert all(rack["source_instance_id"] is None for rack in result["racks"])


def test_anonymous_partial_view_does_not_create_duplicate_when_axis_turns():
    tracker = RackTracker(
        rack_distance_m=1.8, layer_xy_distance_m=1.8,
        min_confirmed_frames=3, use_source_instance_hints=False,
    )
    levels = (0.55, 1.05, 1.55)
    for frame in range(3):
        tracker.update([
            replace(observation(z, 2.0), source_id=100 + frame * 7 + i)
            for i, z in enumerate(levels)
        ], float(frame + 1))
    stable_id = tracker.to_dict("sim_world", 3.0)["racks"][0]["stable_id"]
    tracker.update([
        replace(observation(z, 2.75), source_id=700 + i, axis=(0.9, 0.4359))
        for i, z in enumerate(levels)
    ], 4.0)
    result = tracker.to_dict("sim_world", 4.0)
    assert len(result["racks"]) == 1
    assert result["racks"][0]["stable_id"] == stable_id


def test_anonymous_same_height_fragments_do_not_merge_into_seven_layer_rack():
    tracker = RackTracker(
        rack_distance_m=1.8, group_distance_m=1.1,
        min_confirmed_frames=1, use_source_instance_hints=False,
    )
    observations = [
        replace(observation(z, x), source_id=100 + i)
        for i, (x, z) in enumerate(
            [(2.0, 0.55), (2.0, 1.05), (2.0, 1.55),
             (3.2, 0.55), (3.2, 1.05), (3.2, 1.55)]
        )
    ]
    tracker.update(observations, 1.0)
    assert len(tracker.racks) == 2
    assert sorted(len(rack.layers) for rack in tracker.racks.values()) == [3, 3]
