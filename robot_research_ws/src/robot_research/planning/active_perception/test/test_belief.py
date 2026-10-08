from active_perception.belief import regularize_covariance, TargetBeliefFilter
from active_perception.belief_benchmark import summarize_rows
import numpy as np


def test_repeated_observations_reduce_uncertainty_and_average_position():
    belief = TargetBeliefFilter(process_variance_per_second=0.0)
    noise = np.eye(3) * 0.01
    first = belief.update(np.array((1.0, 0.0, 0.5)), noise, 1.0)
    second = belief.update(np.array((1.02, 0.0, 0.5)), noise, 2.0)

    assert first.accepted
    assert second.accepted
    assert second.snapshot is not None
    assert second.snapshot.accepted_count == 2
    assert np.allclose(second.snapshot.mean, (1.01, 0.0, 0.5))
    assert np.trace(second.snapshot.covariance) < np.trace(noise)


def test_large_innovation_is_rejected_without_moving_belief():
    belief = TargetBeliefFilter(process_variance_per_second=0.0)
    noise = np.eye(3) * 1.0e-4
    belief.update(np.array((1.0, 0.0, 0.5)), noise, 1.0)
    result = belief.update(np.array((3.0, 2.0, 1.5)), noise, 2.0)

    assert not result.accepted
    assert result.reason == "innovation_gate"
    assert result.snapshot is not None
    assert result.snapshot.accepted_count == 1
    assert result.snapshot.rejected_count == 1
    assert np.allclose(result.snapshot.mean, (1.0, 0.0, 0.5))


def test_out_of_order_observation_is_rejected():
    belief = TargetBeliefFilter()
    noise = np.eye(3) * 0.01
    belief.update(np.array((1.0, 0.0, 0.5)), noise, 2.0)
    result = belief.update(np.array((1.0, 0.0, 0.5)), noise, 1.0)
    assert not result.accepted
    assert result.reason == "out_of_order"


def test_covariance_is_symmetrized_and_eigenvalues_are_bounded():
    covariance = np.array(
        ((0.0, 2.0e-4, 0.0), (0.0, 1.0e-4, 0.0), (0.0, 0.0, 2.0))
    )
    result = regularize_covariance(covariance, 1.0e-5, 0.25)
    assert np.allclose(result, result.T)
    eigenvalues = np.linalg.eigvalsh(result)
    assert np.all(eigenvalues >= 1.0e-5 - 1.0e-12)
    assert np.all(eigenvalues <= 0.25 + 1.0e-12)


def test_benchmark_summary_reports_error_and_uncertainty_reduction():
    rows = [
        {"position_error_m": 0.02, "covariance_trace_m2": 0.003},
        {"position_error_m": 0.01, "covariance_trace_m2": 0.001},
    ]
    summary = summarize_rows(rows)
    assert summary["sample_count"] == 2
    assert np.isclose(summary["position_error_rmse_m"], np.sqrt(0.00025))
    assert np.isclose(summary["covariance_trace_reduction_ratio"], 2.0 / 3.0)
