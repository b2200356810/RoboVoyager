#!/bin/bash

gnome-terminal --tab -- roslaunch rosbridge_server rosbridge_websocket.launch
sleep 1
gnome-terminal --tab -- rostopic pub /terminal_topic std_msgs/String "data: 'Hello from terminal! '" --rate 0.5
sleep 1
gnome-terminal --tab -- roslaunch hello_world hello_world.launch
sleep 1
gnome-terminal --tab -- roslaunch video_streaming video_streaming.launch
# sleep 1
# cd frontend/sveltekit/build
# gnome-terminal --tab -- python3 -m http.server 8000