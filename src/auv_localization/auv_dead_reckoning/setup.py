import os
from glob import glob
from setuptools import find_packages, setup

package_name = "auv_dead_reckoning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AUV SLAM contributors",
    maintainer_email="dev@example.com",
    description="DVL+IMU dead-reckoning baseline.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dead_reckoning = auv_dead_reckoning.dead_reckoning_node:main",
        ],
    },
)
