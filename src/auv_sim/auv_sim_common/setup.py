import os
from glob import glob
from setuptools import find_packages, setup

package_name = "auv_sim_common"

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
    description="Simulation-abstraction layer: ground-truth republisher + depth converter.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ground_truth_republisher = auv_sim_common.ground_truth_republisher:main",
            "depth_from_pressure = auv_sim_common.depth_from_pressure:main",
            "sim_dvl = auv_sim_common.sim_dvl:main",
            "sim_usbl = auv_sim_common.sim_usbl:main",
        ],
    },
)
