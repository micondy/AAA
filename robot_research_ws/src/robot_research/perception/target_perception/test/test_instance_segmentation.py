import numpy as np
import pytest
import yaml

from target_perception.instance_segmentation import (
    draw_instance_overlay,
    InstancePrediction,
    parse_torchvision_instances,
    TorchvisionMaskRCNNBackend,
)


def test_overlay_preserves_shape_and_marks_mask_and_box():
    image = np.zeros((8, 10, 3), dtype=np.uint8)
    prediction = InstancePrediction(
        class_index=1,
        class_id="person",
        score=0.95,
        bbox_xyxy=(2.0, 1.0, 7.0, 5.0),
        mask=_output()["masks"][0, 0] > 0.5,
    )
    overlay = draw_instance_overlay(image, [prediction])

    assert overlay.shape == image.shape
    assert overlay.dtype == np.uint8
    assert np.any(overlay != image)
    assert not np.any(image)


def test_overlay_rejects_mismatched_mask():
    prediction = InstancePrediction(1, "person", 0.9, (0, 0, 2, 2), np.ones((2, 2)))
    with pytest.raises(ValueError, match="mask dimensions"):
        draw_instance_overlay(np.zeros((4, 4, 3), dtype=np.uint8), [prediction])


def _output():
    masks = np.zeros((3, 1, 8, 10), dtype=np.float32)
    masks[0, 0, 1:5, 2:7] = 0.9
    masks[1, 0, 2:6, 1:4] = 0.8
    masks[2, 0, 0:2, 0:2] = 0.7
    return {
        "boxes": np.asarray([[2, 1, 7, 5], [1, 2, 4, 6], [0, 0, 2, 2]]),
        "labels": np.asarray([1, 2, 1]),
        "scores": np.asarray([0.95, 0.75, 0.2]),
        "masks": masks,
    }


def test_parser_filters_score_and_preserves_multiple_instances():
    predictions = parse_torchvision_instances(
        _output(),
        ["background", "person", "bicycle"],
        score_threshold=0.5,
    )

    assert [item.class_id for item in predictions] == ["person", "bicycle"]
    assert predictions[0].bbox_xyxy == (2.0, 1.0, 7.0, 5.0)
    assert predictions[0].mask.dtype == np.bool_
    assert int(predictions[0].mask.sum()) == 20


def test_parser_can_select_task_classes():
    predictions = parse_torchvision_instances(
        _output(),
        ["background", "person", "bicycle"],
        allowed_class_indices={2},
    )
    assert len(predictions) == 1
    assert predictions[0].class_index == 2


def test_parser_rejects_malformed_output():
    output = _output()
    del output["masks"]
    with pytest.raises(ValueError, match="missing keys"):
        parse_torchvision_instances(output, ["background", "person", "bicycle"])

    output = _output()
    output["labels"] = np.asarray([1])
    with pytest.raises(ValueError, match="inconsistent"):
        parse_torchvision_instances(output, ["background", "person", "bicycle"])


def test_backend_refuses_missing_weights_without_downloading(tmp_path):
    with pytest.raises(FileNotFoundError, match="weights do not exist"):
        TorchvisionMaskRCNNBackend(tmp_path / "missing.pth")


def test_general_object_config_and_launch_contract():
    package = __import__("pathlib").Path(__file__).parents[1]
    with (package / "config" / "instance_segmentation.yaml").open(
        encoding="utf-8"
    ) as stream:
        params = yaml.safe_load(stream)["instance_segmentation"]["ros__parameters"]
    launch = (package / "launch" / "instance_segmentation.launch.py").read_text(
        encoding="utf-8"
    )

    assert params["allowed_classes"] == ["*"]
    assert "mushroom" not in params["allowed_classes"]
    assert params["weights_path"] == ""
    assert 'executable="instance_segmentation"' in launch
    assert 'DeclareLaunchArgument("weights_path", default_value="")' in launch
