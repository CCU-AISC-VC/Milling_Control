from glob import glob

from setuptools import find_packages, setup

package_name = "milling_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vincent",
    maintainer_email="d13420004@ccu.edu.tw",
    description="Sim2real bridge: play Milling_Control trajectory files on a FollowJointTrajectory controller and record joint states.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "trajectory_player = milling_bridge.trajectory_player:main",
            "joint_state_recorder = milling_bridge.joint_state_recorder:main",
        ],
    },
)
