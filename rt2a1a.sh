#!/bin/bash

gnome-terminal --tab --title="roscore" -- bash -c "source ros.sh; roscore"
gnome-terminal --tab --title="user_interface" -- bash -c "sleep 1; source ros.sh; roslaunch rt2_assignment1 sim.launch"
