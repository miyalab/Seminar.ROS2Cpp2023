from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_prefix = get_package_share_directory("semi_robot_launcher")
    node1 = Node(
        package="semi_robot_core",
        executable="robot_core",
        name="robot",
        remappings=[
            ("~/state", "/robot/state"),
            ("~/cmd_vel", "/robot/cmd_vel")
        ],
        parameters=[join(pkg_prefix, "cfg/robot.yaml")]
    )
    node2 = Node(
        package="semi_robot_controller",
        executable="robot_controller",
        name="controller",
        remappings=[
            ("~/state", "/robot/state"),
            ("~/cmd_vel", "/robot/cmd_vel")
        ],
    )
    return LaunchDescription([
        node1,
        node2
    ])