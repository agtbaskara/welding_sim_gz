# welding_sim_gz
Franka Panda Arm Robot Simulation With Camera for Welding Inspection in Gazebo 

## Software Requirment:
- Ubuntu 18.04
- ROS Melodic
- Notes: Should be compatible with Ubuntu 20.04 + ROS Noetic

## Setup Guide
1. Clone and Build panda_simulator to your workspace (https://github.com/justagist/panda_simulator)
2. Clone and Build this repository to your workspace

## How to Run
1. Run gazebo world (`roslaunch welding_sim_gz panda_world_camera.launch`)
2. Run moveit (`roslaunch welding_sim_gz panda_sim_moveit_camera.launch`)
3. Run simulation_bridge node (`rosrun welding_sim_gz simulation_bridge.py`)
