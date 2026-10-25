from glob import glob

from setuptools import find_packages, setup


package_name = "rack_perception"

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
    description="RGB-D rack geometry, persistent mapping and final alignment.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rack_detector = rack_perception.detector_node:main",
            "rack_yolo_detector = rack_perception.yolo_detector_node:main",
            "rack_geometry = rack_perception.geometry_node:main",
            "rack_map = rack_perception.map_node:main",
            "harvest_planner = rack_perception.planner_node:main",
            "rack_alignment = rack_perception.alignment_node:main",
        ],
    },
)
