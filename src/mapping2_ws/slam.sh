#!/bin/bash

gnome-terminal --tab --title="build my_bot pkg" --working-directory="/home/remon/ros2_ws"  -- bash -c "colcon build --packages-select my_bot "
sleep 6

gnome-terminal --tab --title="source ws "  -- bash -c 'source ~/.bashrc'

gnome-terminal --tab --title="Launch & gazebo" --command="ros2 launch my_bot launch_sim.launch.py world:=/home/remon/ros2_ws/src/my_bot/worlds/cone.world"
sleep 3

gnome-terminal --tab --title="slam toolbox" --command=" ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"
sleep 3

gnome-terminal --tab --title="rviz2" --command="ros2 run rviz2 rviz2 /home/.rviz2/default.rviz  "
sleep 3

gnome-terminal --tab --title="teleop" --command="ros2 run teleop_twist_keyboard teleop_twist_keyboard"


#/home/.rviz2/default.rviz
