# ROS Commands

List of commands for installing and using ROS **Noetic Ninjemys** (on Ubuntu 20.04)

The basic commands are:

```sh
roscore
rosnode # list, info
rostopic # list, info, echo
rosservice # list, info
roslaunch # package_name package_name.launch
```

Double tab is your best friend for anything ROS or Linux related. Try testing out any command with typing a few letters and getting some information (i.e. ros+double_tab cat+double_tab).

##

#### Single Line Install (ROS)

```sh
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

##

#### Source ROS

```sh
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

##

#### Run ROS Master Process

Some third party packages initialize the master process without the need to run roscore.

```sh
roscore
```

##

#### Make New Catkin Workspace

Make a folder for your workspace and put an empty src directory inside it. Then run catkin_make to generate <code>devel</code> and <code>build</code> folders.

```sh
mkdir -p ~/your_workspace/src
catkin_make
```

##

#### Source Your Workspace

This is not strictly necessary but it's for convenience's sake. You won't have to source your project's workspace every new session.

```sh
gedit ~/.bashrc

# Scroll to the bottom and append your workspace after: source /opt/ros/noetic/setup.bash
source ~/your_workspace/devel/setup.bash
```

##

#### Source Terminal Session

```sh
# cd inside ~/your_workspace
source devel/setup.bash
```

##

#### Listen To a Topic

```sh
# roscore must be running in one terminal tab
# then run rostopic echo in another tab
rostopic echo / # hit tab twice to list available/active topics
```

##

#### Create New ROS Node/Package

[CreatingPackage](http://wiki.ros.org/cn/ROS/Tutorials/catkin/CreatingPackage)

> NOTE: MUST BE IN THE SRC OF THE WORKSPACE

```sh
cd ~/your_workspace/src # must cd here and then run catkin_create_pkg
#catkin_create_pkg package_name dependency1 dependency2 language
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
# You can add as many dependencies as you want
# Dependencies and languages can be written out of order, but the package name must be written after catkin_create_pkg
```

##

#### Remove Package

Just delete the folder.

##

#### List Installed Packages

[How do I find the list of installed ROS packages?](http://wiki.ros.org/FAQ#How_do_I_find_the_list_of_installed_ROS_packages.3F)

```sh
rospack list-names
```

##

#### Python Executable

First, you have to make a .py file inside src folder.

```sh
chmod +x your_node.py # Must cd into ~/your_workspace/src/your_package/src
```

##

#### Run Node

```sh
# Must have first created an executable of the file in the node/package
rosrun your_package your_node.py
# rosrun + double tab reveals list of packages that can be run
```

##

#### Bandwidth

You can use tools like rostopic bw to get bandwidth information, which includes the rate and the average message size. While this won't give you the exact size of each message, it can provide an average.

```sh
rostopic bw /video_streaming_topic
```

##

#### If Catkin Workspace Gets Corrupted

```sh
sudo apt-get install --reinstall ros-noetic-catkin
```

##

---

## Dependencies

```sh
# Rosbridge Server
sudo apt install ros-noetic-rosbridge-server
roslaunch rosbridge_server rosbridge_websocket.launch

# Video 4 Linux
sudo apt install v4l-utils
# Check resolution end codec formats supported
v4l2-ctl -d /dev/video0 --list-formats-ext

# OpenCV
sudo apt install opencv

# Shell Build
chmod +x test.sh
# Shell Run
./test.sh

# Serial for GPS
sudo apt-get install ros-noetic-serial
```

##

---

# ROS Commands from other repos

#### rosbag info

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
