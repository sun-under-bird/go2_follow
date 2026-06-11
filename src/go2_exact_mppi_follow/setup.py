from glob import glob

from setuptools import find_packages, setup

package_name = "go2_exact_mppi_follow"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/mppi_config", glob("config/mppi_config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="one1000",
    maintainer_email="one1000@example.com",
    description="EXACT-MPPI style stereo point-cloud and UWB following controller for Unitree Go2.",
    license="GPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go2_uwb_goal_bridge = go2_exact_mppi_follow.go2_uwb_goal_bridge:main",
            "go2_exact_mppi_node = go2_exact_mppi_follow.go2_exact_mppi_node:main",
        ],
    },
)
