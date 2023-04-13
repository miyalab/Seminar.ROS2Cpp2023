from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="tutorial_package",
        executable="helloworld"
    )
    return LaunchDescription([
        node
    ])