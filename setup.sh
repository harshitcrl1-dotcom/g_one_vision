#!/bin/bash
# Clone all dependencies
cd ~
git clone https://github.com/unitreerobotics/unitree_mujoco
git clone https://github.com/unitreerobotics/unitree_ros2
git clone https://github.com/unitreerobotics/unitree_rl_mjlab

# Copy custom scene to mujoco
cp ~/g_one_vision/mujoco_scenes/scene_29dof.xml \
   ~/unitree_mujoco/unitree_robots/g1/

# Copy fixed URDF
cp ~/g_one_vision/urdf/g1_29dof_fixed.urdf \
   ~/unitree_ros2/src/unitree_ros/robots/g1_description/

# Copy and build custom ROS2 package
cp -r ~/g_one_vision/src/g1_joint_state_bridge \
      ~/unitree_ros2/src/
cd ~/unitree_ros2
colcon build --packages-select g1_joint_state_bridge
source install/setup.bash

echo "Setup complete! Add aliases to ~/.bashrc then run g1_mujoco"
