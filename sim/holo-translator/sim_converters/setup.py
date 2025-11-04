from setuptools import find_packages, setup
import os
from glob import glob

package_name = "sim_converters"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ue4",
    maintainer_email="ue4@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dvl_convert = sim_converters.dvl_convert:main",
            "depth_convert = sim_converters.depth_convert:main",
            "gps_convert = sim_converters.gps_convert:main",
            "imu_convert = sim_converters.imu_convert:main",
            "ucommand_bridge = sim_converters.ucommand_bridge:main",
        ],
    },
)
