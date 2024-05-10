# ROS commands

List of commands for installing and using ROS **Noetic Ninjemys** (Ubuntu 20 dependency)

## 1 Line Install

```bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

## Source ROS

```bash
nano ~/.bashrc
source /opt/ros/noetic/setup.bash
```

### Catkin Workspace

Make a folder for your workspace and put empty src directory inside it. Then run catkin_make to generate devel, build folders.

```bash
mkdir -p ~/your_workspace/src
catkin_make
```

## Source the workspace

```bash
nano ~/.bashrc
source ~/your_workspace/devel/setup.bash
```

# Rosbridge server

sudo apt install ros-noetic-rosbridge-server
roslaunch rosbridge_server rosbridge_websocket.launch

# node listener

rostopic echo /first_node_topic

# python executable

chmod +x first_node.py

# send message from terminal to browser

rostopic pub /terminal_topic std_msgs/String "data: 'Hello world! '" -1

# shell build

chmod +x start_ros.sh

#shell start
./start_ros.sh

#IF CATKIN GETS CORRUPTED
sudo apt-get install --reinstall ros-noetic-catkin

##Check video resolution (Ubuntu)
v4l2-ctl -d /dev/video0 --list-formats-ext
##Install with
sudo apt install v4l-utils

##Open CV info for image encoders, video codecs, etc

print(cv2.getBuildInformation())

## setup environment

[Ros launch command not found](https://answers.ros.org/question/264704/ros-launch-command-not-found/)

```sh
echo "source <catkin_ws_dir>/devel/setup.bash" >> ~/.bashrc
~/.bashrc
```

This solves `Command 'roslaunch' not found`.

## list installed packages

[How do I find the list of installed ROS packages?](http://wiki.ros.org/FAQ#How_do_I_find_the_list_of_installed_ROS_packages.3F)

```sh
rospack list-names
```

## catkin_create_pkg

[CreatingPackage](http://wiki.ros.org/cn/ROS/Tutorials/catkin/CreatingPackage)

```sh
cd ~/catkin_ws/src # must cd here and then catkin_create_pkg!
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

## remove package

[catkin: move/remove package and workspace](https://answers.ros.org/question/105576/catkin-moveremove-package-and-workspace/)
Just delete the folder.

## catkin_make && source devel/setup.bash

[roscd: No such package/stack 'beginner_tutorials'](https://answers.ros.org/question/65003/roscd-no-such-packagestack-beginner_tutorials/)

```sh
# in "catkin_ws" directory
# catkin_create_pkg learning_tf2 tf2 tf2_ros roscpp rospy turtlesim
catkin_make
source devel/setup.bash
# roscd learning_tf2
```

This solves `roscd: No such package/stack 'learning_tf2'`.

## catkin_make specific package

[How to build just one package using catkin_make?](https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/)

```sh
catkin_make --only-pkg-with-deps <target_package>
```

## /usr/bin/env: ‘python’: No such file or directory

[Unable to broadcast the turtle position to tf2 , Tutorial: turtle_tf2_demo.launch and getting only one turtle](https://answers.ros.org/question/357423/unable-to-broadcast-the-turtle-position-to-tf2-tutorial-turtle_tf2_demolaunch-and-getting-only-one-turtle/)

```sh
sudo ln -s /usr/bin/python3 /usr/bin/python
```

## [ERROR] [1628307689.337043638]: [registerPublisher] Failed to contact master at [localhost:11311]. Retrying...

This can be solved by running:

```sh
roscore
```

[roscore](http://wiki.ros.org/roscore)

```
roscore is a collection of nodes and programs that are pre-requisites of a ROS-based system.
You must have a roscore running in order for ROS nodes to communicate. It is launched using the roscore command.
```

## Exception thrown:"turtle1" passed to lookupTransform argument source_frame does not exist.

In [Writing a tf2 broadcaster (C++)](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29),
when running:

```sh
roslaunch learning_tf2 start_demo.launch
```

It shows:

```
Failure at 1628308374.969766010
Exception thrown:"turtle" passed to lookupTransform argument source_frame does not exist.
The current list of frames is:
Frame turtle1 exists with parent world.
```

It turns out that we should create a folder named `launch` and put `start_demo.launch` there.

```sh
# in catkin_ws/src/learning_tf2
mkdir launch
vim launch/start_demo.launch # fill it
# and then roslaunch again
roslaunch learning_tf2 start_demo.launch
```

It will successfully outputs:

```
At time 1628308379.856
- Translation: [6.698, 4.107, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.606, 0.796]
            in RPY (radian) [0.000, 0.000, -1.301]
            in RPY (degree) [0.000, 0.000, -74.531]
```

## rosbag info

[Recording and playing back data](http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data)

```sh
rosbag info xxx.bag
```

To find location of a package
rospack find roscpp
To navigate to a package
roscd roscpp
roscd roscpp/cmake
roscd log
To find contents of a package
rosls roscpp_tutorials
To create a workspace
catkin_init_workspace

To find available packages
apt-cache search ros-indigo

To create a package
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
To build a package
catkin_make
Sourcing newly built package
. ~/catkin_ws/devel/setup.bash
Checking dependencies of a package
rospack depends1 beginner_tutorials

Error while running roscore, change permission of ros folder using below
sudo chown -R <your_username> ~/.ros
List all ROS Nodes
rosnode list
Getting info about a node
rosnode info /rosout
Rosrun allows you to use the package name to directly run a node within a package
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
Running node with custom name  
 rosrun turtlesim turtlesim_node \_\_name:=my_turtle  
Pinging a node to see if it is up
rosnode ping my_turtle

Installing RQT graph
sudo apt-get install ros-indego-rqt
Running graph
rosrun rqt_graph rqt_graph

Rostopic help
rostopic -h
rostopic list -h
Subscribing to rostopic
rostopic echo /turtle1/cmd_vel
Checking type of rostopic
rostopic type /turtle1/cmd_vel
Seeing details of message
rosmsg show geometry_msgs/Twist
To show type and message
rostopic type /turtle1/cmd_vel | rosmsg show
PUBLISHING custom message
rostopic pub [topic] [msg_type] [args]
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'

Continuously publish at 1Hz for keep on moving
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
How fast node is publishing data
rostopic hz /turtle1/pose
To plot rostopic data in chart
rosrun rqt_plot rqt_plot

List Rosservices
rosservice list
rosservice -help
rosservice call /clear
rosservice call /spawn 2 2 0.2 ""
rosservice type /spawn| rossrv show

Rosparam allows you to store and manipulate data on the ROS Parameter Server. Uses YAML for syntax
rosparam --help
rosparam set /background_r 150
rosservice call /clear
List all data in rosparam server
rosparam get /
rosparam dump params.yaml
rosparam load params.yaml
Loading to another namespace and using  
 rosparam load params.yaml copy
rosparam get /copy/background_b

Running console
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
Hitting wall to simulate error
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

All rqt tools
rqt
Only graph
rqt_graph

msg: msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
srv: an srv file describes a service. It is composed of two parts: a request and a response.  
Copying service from another package to working package
roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv

Showing service details
rossrv show <service type>
rossrv show beinner_tutorials/AddTwoInts
rossrv show AddTwoInts
Help for ros messages
rosmsg -h
If you are building C++ nodes which use your new messages, you will also need to declare a dependency between your node and your message, as described in the catkin msg/srv build documentation.
Recording published messages
mkdir ~/bagfiles
cd ~/bagfiles
rosbag record -a
Getting info about recorded messages
rosbag info bagfilename
Playing back messages  
 rosbag play bagfilename  
Using package
roscore
rosrun sound_play soundplay_node.py
rosrun sound_play say.py 'Hello world'

echo $ROS_PACKAGE_PATH will give paths where ROS packages are installed

rostopic bw /video_streaming_topic
ROS Tools:

You can use tools like rostopic bw to get bandwidth information, which includes the rate and the average message size. While this won't give you the exact size of each message, it can provide an average.

DEPENDENCIES
sudo apt-get install ros-noetic-serial