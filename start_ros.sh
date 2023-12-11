#!/bin/bash

gnome-terminal --tab -- roslaunch rosbridge_server rosbridge_websocket.launch
sleep 1
gnome-terminal --tab -- rosrun first_node first_node.py
sleep 1
gnome-terminal --tab -- roslaunch multimedia multimedia_script.launch
sleep 1
cd frontend/basic
gnome-terminal --tab -- python3 -m http.server 8000