"""Start the OCS2 MPC node for the Husky + UR5e MuJoCo bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory("husky_ur5e_description"))
    codegen = package / "ocs2" / "codegen"
    return LaunchDescription([
        Node(
            package="ocs2_mobile_manipulator_ros",
            executable="mobile_manipulator_mpc_node",
            name="mobile_manipulator_mpc",
            output="screen",
            parameters=[{
                "taskFile": str(package / "ocs2" / "task.info"),
                "urdfFile": str(package / "ocs2" / "husky_ur5e.urdf"),
                "libFolder": str(codegen),
            }],
        ),
    ])
