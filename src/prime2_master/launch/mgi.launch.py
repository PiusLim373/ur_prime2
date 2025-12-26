from launch import LaunchDescription
from launch_ros.actions import Node

# from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            "ur",
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )


    

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="mgi_node",
        package="prime2_master",
        executable="mgi_node",
        output="screen",
        parameters=[
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_demo])
