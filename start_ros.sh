#!/bin/bash

# Start catkin_make in the current terminal tab
gnome-terminal --tab -- bash -c "catkin_make; exec bash"

# Wait for a few seconds to allow catkin_make to finish building (adjust the duration if needed)
sleep 2

# Start roscore
gnome-terminal --tab -- roscore

# Wait for a few seconds to allow roscore to start (adjust the duration if needed)
sleep 3

# Open a new tab and launch rosbridge
gnome-terminal --tab -- roslaunch rosbridge_server rosbridge_websocket.launch

# Wait for a few seconds to allow rosbridge to start (adjust the duration if needed)
sleep 2

# Open a new tab and run first_node.py
gnome-terminal --tab -- rosrun first_node first_node.py

