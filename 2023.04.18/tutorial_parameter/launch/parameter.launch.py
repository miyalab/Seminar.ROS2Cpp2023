from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_prefix = get_package_share_directory("tutorial_parameter")
    node = Node(
        package="tutorial_parameter",
        executable="parameter",
        parameters=[join(pkg_prefix, "cfg/parameter.yaml")]
    )
    return LaunchDescription([
        node
    ])