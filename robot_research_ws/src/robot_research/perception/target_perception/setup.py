from glob import glob
from setuptools import find_packages, setup


package_name = "target_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Mobile Robot",
    maintainer_email="maintainer@example.com",
    description="Camera-independent RGB-D target observations for active perception.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "benchmark_recorder = target_perception.benchmark_node:main",
            "color_depth_detector = target_perception.color_depth_detector_node:main",
            "distance_benchmark_recorder = target_perception.distance_benchmark_node:main",
            "general_object_benchmark = target_perception.general_object_benchmark_node:main",
            "instance_segmentation = target_perception.instance_segmentation_node:main",
            "instance_tracking = target_perception.instance_tracking_node:main",
        ],
    },
)
