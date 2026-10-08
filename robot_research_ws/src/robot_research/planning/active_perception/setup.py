from glob import glob
from setuptools import find_packages, setup


package_name = "active_perception"

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
    description="Target belief and next-best-view components for mobile manipulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "belief_benchmark = active_perception.belief_benchmark_node:main",
            "nbv_executor = active_perception.nbv_executor_node:main",
            "nbv_planner = active_perception.nbv_planner_node:main",
            "target_selector = active_perception.target_selector_node:main",
            "target_belief = active_perception.target_belief_node:main",
        ],
    },
)
