#!/bin/bash
#gnome-terminal --tab --title="build my_bot pkg" --working-directory="/home/saqr/ros2_ws"  -- bash -c "colcon build --packages-select my_bot "
#sleep 4

#gnome-terminal --tab --title="source ws "  -- bash -c 'source ~/.bashrc'
#sleep 4

gnome-terminal --tab --title="Launch & gazebo" -- bash -c "ros2 launch my_bot launch_sim.launch.py world:=/home/saqr/ros2_ws/src/my_bot/worlds/cone.world"
sleep 6

gnome-terminal --tab --title="nav2" -- bash -c "ros2 launch turtlebot3_navigation2  navigation2.launch.py  use_sim_time:=True map:=maps/new_map.yaml"

#gnome-terminal --tab --title="slam toolbox" --command=" ros2 launch slam_toolbox online_async_launch.py map:=/home/saqr/ros2_ws/src/my_bot/my_map_save.yaml use_sim_time:=True"
#sleep 3

#gnome-terminal --tab --title="Navigation" --command="ros2 launch nav2_bringup navigation_launch.py map:=/home/saqr/ros2_ws/my_map_save.yaml"
#sleep 3

#gnome-terminal --tab --title="rviz2" --command="ros2 run rviz2 rviz2  /home/.rviz2/s1.rviz"
#sleep 3

#gnome-terminal --tab --title="teleop" --command="ros2 run teleop_twist_keyboard teleop_twist_keyboard"




#gnome-terminal --tab --title="build my_bot pkg" --working-directory="/home/saqr/ros2_ws"  -- bash -c "colcon build --packages-select my_bot "
#sleep 4

#gnome-terminal --tab --title="source ws "  -- bash -c 'source ~/.bashrc'
#sleep 4
