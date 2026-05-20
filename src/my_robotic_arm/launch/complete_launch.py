import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Package and Robot Names
    robot_name = "config_arm_robot"
    main_pkg = "my_robotic_arm"
    moveit_config_pkg = "gp_arm_moveit"

    # 2. Load MoveIt Configuration
    # This gathers the URDF, SRDF, Kinematics, and Joint Limits
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_config_pkg)
        .to_moveit_configs()
    )

    # 3. Gazebo Simulation Launch (Empty World)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. Spawn the Robot in Gazebo
    # It pulls the 'robot_description' parameter from the RSP node below
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    # 5. Robot State Publisher
    # This broadcasts the static and dynamic TFs to /tf
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': True}
        ],
    )

    # 6. MoveGroup Node (The IK & Planning Brain)
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}
        ],
    )

    # 7. RViz with MoveIt Config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
    )

    # 8. Controller Spawners
    # These must be called after the robot is spawned in Gazebo
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )

    return LaunchDescription([
        # Start Gazebo and RSP immediately
        gazebo,
        rsp,
        spawn_robot,
        
        # Start MoveGroup and RViz
        run_move_group_node,
        rviz_node,

        # Delay spawning controllers slightly to ensure Gazebo is ready
        TimerAction(
            period=2.0,
            actions=[jsb_spawner, arm_controller_spawner]
        ),
    ])