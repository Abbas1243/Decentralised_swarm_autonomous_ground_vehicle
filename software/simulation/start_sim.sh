#!/bin/bash
echo "Starting Zenoh router..."
gnome-terminal -- bash -c "ros2 run rmw_zenoh_cpp rmw_zenohd; exec bash"

sleep 2
echo "Starting Gazebo..."
gnome-terminal -- bash -c "source ~/FinalYearProject/install/setup.bash && ros2 launch swarm_bot_gazebo gazebo.launch.py; exec bash"

sleep 5
echo "Spawning robot_1..."
gnome-terminal -- bash -c "source ~/FinalYearProject/install/setup.bash && ros2 launch swarm_bot_gazebo spawn_robot.launch.py namespace:=robot_1 x_pos:=0.0 y_pos:=0.0; exec bash"

sleep 3
echo "Starting SLAM + Nav2..."
gnome-terminal -- bash -c "source ~/FinalYearProject/install/setup.bash && ros2 launch swarm_bot_navigation slam_nav.launch.py namespace:=robot_1; exec bash"

echo "All systems starting. Wait for 'Managed nodes are active' then run RViz."
