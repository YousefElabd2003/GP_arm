#!/bin/bash

# 1. GPU / Rendering Settings
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export GZ_RENDERING_BACKEND=ogre2
export QT_OFFSCREEN_PIPELINE_ALWAYS_SKIP=1
export OGRE_RTT_MODE=FBO

# 2. Source Environments
source /opt/ros/humble/setup.bash
source ~/gp_arm/install/setup.bash

# 3. FIX: Mesh Paths for Gazebo
# This tells Gazebo to look inside your src folder for the meshes
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/gp_arm/src:~/gp_arm/install/my_robotic_arm/share

# Set ROS 2 Network communication for WSL2 stability
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
# 4. Refresh Daemon
ros2 daemon stop
ros2 daemon start

# 5. Launch the Full Stack
ros2 launch my_robotic_arm complete_launch.py