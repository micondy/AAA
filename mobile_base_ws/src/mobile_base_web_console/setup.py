from glob import glob
from setuptools import find_packages, setup

package_name = "mobile_base_web_console"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="micondy",
    maintainer_email="2637781202@qq.com",
    description="Mobile Base LAN web console",
    license="Apache-2.0",
    entry_points={"console_scripts": ["web_console = mobile_base_web_console.web_console:main"]},
)
