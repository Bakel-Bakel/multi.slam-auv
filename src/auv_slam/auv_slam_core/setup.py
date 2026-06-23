import os
from glob import glob
from setuptools import find_packages, setup

package_name = "auv_slam_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AUV SLAM contributors",
    maintainer_email="dev@example.com",
    description="GTSAM/iSAM2 factor-graph integrator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "slam_core = auv_slam_core.slam_core_node:main",
        ],
    },
)
