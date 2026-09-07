from glob import glob

from setuptools import find_packages, setup

package_name = "turtlebot3_multimodal"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FengDK666",
    maintainer_email="maintainers@example.com",
    description="Safe structured multimodal command layer for TurtleBot3",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safe_controller = turtlebot3_multimodal.ros_node:main",
            "velocity_gate = turtlebot3_multimodal.gate_node:main",
            "evaluate_commands = turtlebot3_multimodal.evaluate_cli:main",
            "navigation_evaluator = turtlebot3_multimodal.navigation_evaluator:main",
        ],
    },
)
