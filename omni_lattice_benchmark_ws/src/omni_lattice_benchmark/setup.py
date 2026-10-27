from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "omni_lattice_benchmark"


def files(pattern: str) -> list[str]:
    return [str(path) for path in Path().glob(pattern) if path.is_file()]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", files("config/*")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/lattice", files("lattice/*")),
        (f"share/{package_name}/maps", files("maps/*")),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="micondy",
    maintainer_email="2637781202@qq.com",
    description="Deterministic Nav2 planner benchmarks for a rectangular 4WIS base.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "generate_benchmark_maps = omni_lattice_benchmark.map_generator:main",
            "generate_coupled_lattices = omni_lattice_benchmark.lattice_generator:main",
            "fetch_official_omni_lattice = omni_lattice_benchmark.official_assets:main",
            "planner_benchmark = omni_lattice_benchmark.planner_benchmark:main",
            "visualize_plan = omni_lattice_benchmark.visualize_plan:main",
        ],
    },
)
