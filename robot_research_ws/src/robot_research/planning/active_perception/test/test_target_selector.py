from active_perception.target_selector import TargetCandidate, TargetSelector
import pytest


def _candidate(object_id, class_id, score, x):
    return TargetCandidate(object_id, class_id, score, (x, 0.0, 1.0))


def test_highest_score_candidate_is_acquired_deterministically():
    selector = TargetSelector(target_class="cup", min_score=0.5)
    result = selector.update(
        [_candidate("cup_0001", "cup", 0.7, 0.5),
         _candidate("cup_0002", "cup", 0.9, 1.0)]
    )
    assert result.selected_id == "cup_0002"
    assert result.visible
    assert result.selection_changed


def test_lock_does_not_jump_to_higher_score_object():
    selector = TargetSelector(target_class="cup")
    selector.update([_candidate("cup_0001", "cup", 0.8, 0.5)])
    result = selector.update(
        [_candidate("cup_0001", "cup", 0.6, 0.5),
         _candidate("cup_0002", "cup", 0.99, 0.6)]
    )
    assert result.selected_id == "cup_0001"
    assert result.state == "tracking"
    assert not result.selection_changed


def test_short_loss_keeps_id_without_publishing_candidate():
    selector = TargetSelector(target_class="bottle", max_missed_frames=2)
    selector.update([_candidate("bottle_0001", "bottle", 0.9, 0.5)])
    first_loss = selector.update([])
    second_loss = selector.update(
        [_candidate("bottle_0002", "bottle", 0.99, 0.6)]
    )
    assert first_loss.selected_id == "bottle_0001"
    assert second_loss.selected_id == "bottle_0001"
    assert not second_loss.visible
    assert second_loss.candidate is None


def test_expired_lock_can_acquire_another_matching_object():
    selector = TargetSelector(target_class="book", max_missed_frames=1)
    selector.update([_candidate("book_0001", "book", 0.9, 0.5)])
    selector.update([])
    result = selector.update([_candidate("book_0002", "book", 0.8, 0.6)])
    assert result.selected_id == "book_0002"
    assert result.selection_changed


def test_explicit_id_never_falls_back_to_another_object():
    selector = TargetSelector(target_id="cup_0007", max_missed_frames=0)
    result = selector.update([_candidate("cup_0008", "cup", 0.99, 0.4)])
    assert result.selected_id is None
    assert result.state == "unavailable"


def test_reconfiguration_clears_previous_lock():
    selector = TargetSelector(target_class="cup")
    selector.update([_candidate("cup_0001", "cup", 0.9, 0.5)])
    selector.configure("", "bottle", 0.0, 5)
    result = selector.update([_candidate("bottle_0001", "bottle", 0.8, 0.5)])
    assert result.selected_id == "bottle_0001"
    assert result.selection_changed


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError, match="min_score"):
        TargetSelector(min_score=1.1)
