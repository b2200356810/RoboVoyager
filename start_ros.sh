#!/bin/bash

gnome-terminal --tab -- roslaunch rosbridge_server rosbridge_websocket.launch
sleep 1
gnome-terminal --tab -- rosrun hello_world_node hello_world.py
sleep 1
gnome-terminal --tab -- roslaunch video_streaming video_streaming.launch
sleep 1
cd frontend/basic
gnome-terminal --tab -- python3 -m http.server 8000