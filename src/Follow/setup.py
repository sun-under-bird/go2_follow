from pathlib import Path

from setuptools import find_packages, setup

package_name = "go2_dynamic_follow_avoidance"
package_root = Path(__file__).parent.resolve()


def package_files(pattern):
    return [str(path) for path in sorted(package_root.glob(pattern))]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [str(package_root / "resource" / package_name)],
        ),
        ("share/" + package_name, [str(package_root / "package.xml")]),
        ("share/" + package_name + "/launch", package_files("launch/*.launch.py")),
        ("share/" + package_name + "/config", package_files("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="one1000",
    maintainer_email="one1000@example.com",
    description="Dynamic ONE1000 follow and stereo obstacle avoidance pipeline for Unitree Go2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follow_goal_node = go2_dynamic_follow_avoidance.follow_goal_node:main",
            "local_path_planner = go2_dynamic_follow_avoidance.local_path_planner:main",
            "follow_path_action_client = go2_dynamic_follow_avoidance.follow_path_action_client:main",
            "simple_follow_controller = go2_dynamic_follow_avoidance.simple_follow_controller:main",
            "safety_mux = go2_dynamic_follow_avoidance.safety_mux:main",
        ],
    },
)
