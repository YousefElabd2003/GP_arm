import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Build MoveIt Configuration
    # This builder automates loading the URDF, SRDF, and YAML configs.
    moveit_config = (
        MoveItConfigsBuilder("config_arm_robot", package_name="gp_arm_moveit")
        .robot_description(file_path="config/config_arm_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/config_arm_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # FIXES THE CRASH
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # 2. Define the MoveGroup Node
    # This is the "brain" that hosts the /move_group action server.
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # 3. Define the Planning Scene Monitor
    # This ensures MoveIt knows where the robot is in Gazebo.
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    return LaunchDescription(
        [
            run_move_group_node,
        ]
    )