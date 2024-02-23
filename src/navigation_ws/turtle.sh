#!/bin/bash


gnome-terminal --tab --title="Launch & gazebo" --command="ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
sleep 5

gnome-terminal --tab --title="nav2" --command="ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True"
sleep 5

gnome-terminal --tab --title="slam toolbox" --command="ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"
sleep 5


gnome-terminal --tab --title="rviz2" --command="ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
sleep 5


gnome-terminal --tab --title="teleop" --command="ros2 run turtlebot3_teleop teleop_keyboard"