from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(
        package="tutorial_topic",
        executable="publisher",
        name="pub",
        remappings=[
            ("/number", "/number1")
        ]
    )
    node2 = Node(
        package="tutorial_topic",
        executable="subscriber",
        name="sub",
        remappings=[
            ("/number", "/number1")
        ]
    )
    return LaunchDescription([
        node1,
        node2
    ])