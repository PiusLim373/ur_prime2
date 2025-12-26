from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prime2_master_dir = PathJoinSubstitution([FindPackageShare("prime2_master"), "launch"])

    move_group_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([prime2_master_dir, "/mgi.launch.py"])
    )

    charuco_detector_node = Node(
        package="prime2_master",
        executable="charuco_detector.py",
        name="charuco_detector_node",
        output="screen",
    )

    servo_master_node = Node(
        package="prime2_master",
        executable="servo_master.py",
        name="servo_master_node",
        output="screen",
    )

    master_controller_node = Node(
        package="prime2_master",
        executable="master_controller.py",
        name="master_controller_node",
        output="screen",
    )

    return LaunchDescription(
        [
            move_group_interface,
            charuco_detector_node,
            servo_master_node,
            master_controller_node,
        ]
    )
