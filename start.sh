#!/bin/bash

gnome-terminal --tab -- bash -c "cd RoboVoyager/frontend/sveltekit && npm run lan"
sleep 1
gnome-terminal --tab -- roslaunch rosbridge_server rosbridge_websocket.launch
sleep 1
gnome-terminal --tab -- rosrun sensors sensors.py
sleep 1
gnome-terminal --tab -- rosrun video_streaming video_streaming_script.py
sleep 1
gnome-terminal --tab -- rosrun data_saver data_SAVER.py
sleep 1
gnome-terminal --tab -- bash -c '
expect <<EOF
  spawn sudo chmod 666 /dev/ttyACM0
  expect "password for *:"
  send "root\r"
  expect eof
EOF
exec bash'
gnome-terminal --tab -- roslaunch controls controls.launch
sleep 1
gnome-terminal --tab -- rostopic echo /cmd_vel
sleep 1
gnome-terminal --tab -- rosrun yolo yolo_script.py
sleep 1
gnome-terminal --tab -- rosrun segmentation segmentation_script.py


# gnome-terminal --tab -- rostopic pub /terminal_topic std_msgs/String "data: 'Hello from terminal! '" --rate 0.5
# gnome-terminal --tab -- python3 -m http.server 8000
