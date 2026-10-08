from setuptools import find_packages, setup


package_name = "experiment_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", ["config/rosbag_qos.yaml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/monitor_camera.launch.py",
                "launch/play_camera.launch.py",
                "launch/record_camera.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="micondy",
    maintainer_email="2637781202@qq.com",
    description="Repeatable rosbag recording and sensor health checks.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_health_monitor = experiment_tools.camera_health_monitor:main",
        ],
    },
)
