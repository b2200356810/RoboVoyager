#!/bin/bash

gnome-terminal --tab -- roslaunch rosbridge_server rosbridge_websocket.launch
gnome-terminal --tab -- rosrun first_node first_node.py
