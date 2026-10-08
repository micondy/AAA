from glob import glob
from setuptools import find_packages, setup


package_name = "mobile_manipulator_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Mobile Robot",
    maintainer_email="maintainer@example.com",
    description="Profile-based bringup and stable interfaces for the mobile manipulator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "base_io_adapter = mobile_manipulator_bringup.base_io_adapter:main",
            "apply_task_scene = mobile_manipulator_bringup.task_scene_node:main",
            "validate_moveit_collisions = mobile_manipulator_bringup.moveit_collision_validation:main",
        ],
    },
)
