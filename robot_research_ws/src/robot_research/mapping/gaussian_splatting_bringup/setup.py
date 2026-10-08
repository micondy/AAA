from setuptools import find_packages, setup


package_name = "gaussian_splatting_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/capture.launch.py",
                "launch/real_capture.launch.py",
                "launch/real_mobile_mapping.launch.py",
                "launch/sim_capture.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="micondy",
    maintainer_email="2637781202@qq.com",
    description="ROS RGB-D and pose capture for Gaussian splatting datasets.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dataset_recorder = gaussian_splatting_bringup.dataset_recorder:main",
            "train_gaussian = gaussian_splatting_bringup.train_gaussian:main",
            "export_gaussian = gaussian_splatting_bringup.export_gaussian:main",
        ],
    },
)
