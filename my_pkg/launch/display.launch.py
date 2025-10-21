from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = get_package_share_directory("my_pkg")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(pkg_path, "description", "robot_all.urdf.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PathJoinSubstitution([pkg_path, "worlds", "empty_gz.world"])
        }.items(),
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            ),
            Node(package="rviz2", executable="rviz2", name="rviz2", output="screen"),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-name",
                    "bot",
                    "-allow_renaming",
                    "true",  # permet de faire le robot dans gz sim
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.1",
                    "-R",
                    "0.0",
                    "-P",
                    "0.0",
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": os.path.join(pkg_path, "config", "bridge.yaml"),
                        "qos_overrides./tf_static.publisher.durability": "transient_local",
                    }
                ],
                output="screen",
            ),
            gz_sim,
        ]
    )
