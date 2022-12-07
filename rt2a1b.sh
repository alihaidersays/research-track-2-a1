#!/bin/bash

gnome-terminal --tab --title="roscore" -- bash -c "source ros.sh; roscore"
gnome-terminal --tab --title="user_interface" -- bash -c "sleep 1; source ros.sh; roslaunch rt2_assignment1 sim_ros2_bridge.launch"
gnome-terminal --tab --title="position_server" -- bash -c "source ros2.sh sleep 2; ros2 launch rt2a1_ros2 ros2_launch.py"
gnome-terminal --tab --title="bridge" -- bash -c "source ros12.sh; sleep 2; ros2 run ros1_bridge dynamic_bridge"
