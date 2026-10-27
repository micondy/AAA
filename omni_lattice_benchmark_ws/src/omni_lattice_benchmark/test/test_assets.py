import json
from pathlib import Path

import yaml

from omni_lattice_benchmark.lattice_generator import generate
from omni_lattice_benchmark.map_generator import GENERATORS


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_four_deterministic_maps_are_declared():
    assert set(GENERATORS) == {
        "open_space",
        "l_corridor",
        "dense_obstacles",
        "narrow_irregular",
    }
    scenarios = yaml.safe_load((PACKAGE_ROOT / "config" / "scenarios.yaml").read_text())
    assert set(scenarios["scenarios"]) == set(GENERATORS)
    assert scenarios["map_resolution"] == 0.05


def test_ablation_lattices_only_change_coupling():
    sequential = generate(coupled=False)
    coupled = generate(coupled=True)
    assert sequential["lattice_metadata"]["num_of_headings"] == 16
    assert coupled["lattice_metadata"]["num_of_headings"] == 16
    assert sequential["lattice_metadata"]["number_of_trajectories"] == 160
    assert coupled["lattice_metadata"]["number_of_trajectories"] == 416

    moving_and_turning = [
        primitive
        for primitive in coupled["primitives"]
        if primitive["trajectory_length"] > 0.0
        and primitive["start_angle_index"] != primitive["end_angle_index"]
    ]
    assert len(moving_and_turning) == 256


def test_generated_assets_have_expected_metadata():
    for name in ("omni_sequential_16_5cm.json", "omni_coupled_16_5cm.json"):
        document = json.loads((PACKAGE_ROOT / "lattice" / name).read_text())
        assert document["lattice_metadata"]["motion_model"] == "omni"
        assert document["lattice_metadata"]["grid_resolution"] == 0.05
        assert len(document["primitives"]) == document["lattice_metadata"]["number_of_trajectories"]

    official = json.loads(
        (PACKAGE_ROOT / "lattice" / "nav2_official_omni_16_5cm_r0.5.json").read_text()
    )
    assert official["lattice_metadata"]["motion_model"] == "omni"
    assert official["lattice_metadata"]["number_of_trajectories"] == 144


def test_planner_profiles_do_not_modify_production_nav2():
    navfn = yaml.safe_load((PACKAGE_ROOT / "config" / "planner_navfn.yaml").read_text())
    smac = yaml.safe_load((PACKAGE_ROOT / "config" / "planner_smac_lattice.yaml").read_text())
    assert navfn["planner_server"]["ros__parameters"]["GridBased"]["plugin"].endswith("NavfnPlanner")
    assert smac["planner_server"]["ros__parameters"]["GridBased"]["plugin"].endswith("SmacPlannerLattice")
    assert "__LATTICE_FILE__" in (PACKAGE_ROOT / "config" / "planner_smac_lattice.yaml").read_text()
