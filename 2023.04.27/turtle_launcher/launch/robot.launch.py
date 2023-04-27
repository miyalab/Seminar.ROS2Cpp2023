from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_prefix = get_package_share_directory("turtle_launcher")
    node1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtle1",
    )
    node2 = Node(
        package="turtle_locator",
        executable="locator",
        name="locator",
        remappings=[
            ("/robot/pose", "/turtle1/pose"),
            ("~/location", "/locator/location")
        ],
        parameters=[join(pkg_prefix, "cfg/robot.yaml")]
    )
    node3 = Node(
        package="turtle_path_planning",
        executable="path_planning",
        name="path_planning",
        remappings=[
            ("/locator/location", "/locator/location"),
            ("~/target_pose",     "/path_planning/target_pose")
        ],
    )
    node4 = Node(
        package="turtle_path_following",
        executable="path_following",
        name="path_following",
        remappings=[
            ("/locator/location",          "/locator/location"),
            ("/path_planning/target_pose", "/path_planning/target_pose"),
            ("~/cmd_vel",                  "/turtle1/cmd_vel")
        ],
    )
    
    return LaunchDescription([
        node1,
        node2,
        node3,
        node4,
    ])