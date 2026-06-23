import os
from glob import glob
from setuptools import find_packages, setup

package_name = "auv_evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AUV SLAM contributors",
    maintainer_email="dev@example.com",
    description="evo ATE/RPE evaluation + rosbag regression harness.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "trajectory_logger = auv_evaluation.trajectory_logger:main",
            "run_evo = auv_evaluation.run_evo:main",
        ],
    },
)
