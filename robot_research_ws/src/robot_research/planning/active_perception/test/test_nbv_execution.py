import math
from types import SimpleNamespace

from active_perception.nbv_execution import make_belief_snapshot, summarize_belief_change
from active_perception.nbv_executor_node import target_replacement_detected
import pytest


def test_target_loss_is_not_misclassified_as_target_replacement():
    assert not target_replacement_detected("cup_0001", "")
    assert not target_replacement_detected("cup_0001", "cup_0001")
    assert target_replacement_detected("cup_0001", "bottle_0002")


def _belief(trace_values=(0.03, 0.02, 0.01), position=(1.0, 2.0, 3.0)):
    covariance = [0.0] * 36
    covariance[0], covariance[7], covariance[14] = trace_values
    point = SimpleNamespace(x=position[0], y=position[1], z=position[2])
    return SimpleNamespace(
        pose=SimpleNamespace(
            covariance=covariance,
            pose=SimpleNamespace(position=point),
        )
    )


def test_paired_belief_summary_reports_reduction_and_new_observations():
    before = make_belief_snapshot(_belief(), 100, True)
    after = make_belief_snapshot(
        _belief((0.015, 0.01, 0.005), (1.01, 2.0, 3.0)), 112, True
    )
    summary = summarize_belief_change(before, after)
    assert summary["new_observations"] == 12
    assert summary["covariance_trace_reduction"] == pytest.approx(0.03)
    assert summary["covariance_trace_reduction_ratio"] == pytest.approx(0.5)
    assert summary["belief_position_shift_m"] == pytest.approx(0.01)


def test_invalid_covariance_trace_is_rejected():
    with pytest.raises(ValueError, match="covariance trace"):
        make_belief_snapshot(_belief((math.nan, 0.0, 0.0)), 1, False)
