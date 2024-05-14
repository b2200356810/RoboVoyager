#!/bin/bash

gnome-terminal --tab -- roslaunch rosbridge_server rosbridge_websocket.launch
sleep 1
gnome-terminal --tab -- rosrun video_streaming video_streaming.py
# sleep 1
# gnome-terminal --tab -- rostopic pub /terminal_topic std_msgs/String "data: 'Hello from terminal! '" --rate 0.5
# sleep 1

# cd frontend/sveltekit/build
# gnome-terminal --tab -- python3 -m http.server 8000
# rosrun multimedia_capture multimedia_capture.py
# roslaunch robot_model gazebo.launch