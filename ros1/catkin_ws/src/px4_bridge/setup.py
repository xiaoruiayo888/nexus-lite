from setuptools import setup

setup(
    name="px4_bridge",
    version="0.1.0",
    description="Lightweight ROS1 (MAVROS) PX4 control bridge",
    packages=["px4_bridge"],
    package_dir={"": "src"},
    install_requires=["PyYAML", "numpy"],
)
