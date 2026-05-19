from glob import glob
from setuptools import find_packages, setup

package_name = "go2_dynamic_follow_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/behavior_trees", glob("behavior_trees/*.xml")),
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
            "nav2_dynamic_follow_client = go2_dynamic_follow_avoidance.nav2_dynamic_follow_client:main",
            "simple_follow_controller = go2_dynamic_follow_avoidance.simple_follow_controller:main",
            "safety_mux = go2_dynamic_follow_avoidance.safety_mux:main",
        ],
    },
)
