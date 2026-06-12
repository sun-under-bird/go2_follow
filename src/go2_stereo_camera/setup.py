from glob import glob
from setuptools import setup

package_name = "go2_stereo_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="one1000",
    maintainer_email="one1000@example.com",
    description="Stitched stereo camera splitter for Go2 follow avoidance.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "stereo_split_node = go2_stereo_camera.stereo_split_node:main",
        ],
    },
)
