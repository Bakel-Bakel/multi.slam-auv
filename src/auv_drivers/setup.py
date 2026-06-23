from setuptools import find_packages, setup

package_name = "auv_drivers"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AUV SLAM contributors",
    maintainer_email="dev@example.com",
    description="Sim-to-real driver contract stubs.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "driver_stub = auv_drivers.driver_stub:main",
        ],
    },
)
