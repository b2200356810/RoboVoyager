<div align="center">
     <a href="https://duygusenaozturk.github.io/">
        <img src="https://github.com/emir-masinovic/RoboVoyager/assets/80472569/0e554785-d310-44ef-bde4-ccb1882bd928" alt="#">
     </a>
     <br>
     <br>
     <p>RoboVoyager is a website interface and a group of Robot Operating System (ROS) nodes that lets you control a robot and monitor its sensors</p>
     <a href="http://opensource.org/licenses/MIT" alt="#"> <img alt="Static Badge" src="https://img.shields.io/badge/license-MIT-blue">
     </a>
</div>

<p align="center">
  <a href="#demo">Demo</a>&nbsp;&nbsp;
  <a href="#key-features">Key Features</a>&nbsp;&nbsp;
  <a href="#how-it-works">How It Works</a>&nbsp;&nbsp;
  <a href="#how-to-use">How To Use</a>&nbsp;&nbsp;
  <a href="#related">Related</a>&nbsp;&nbsp;
  <a href="#license">License</a>
</p>

---

## Demo

A demo of the interface for presentation purposes has been deployed: https://www.robovoyager.online/robot. It doesn't have access to the robot, as we only made it work with LAN connection

<br>

## Key Features

<div align="center">
  <img src="https://github.com/emir-masinovic/RoboVoyager/assets/80472569/d77334c1-efbb-428d-8e3b-a4f0797129a9" alt="Centered Image">
</div>

<h4>🕹️ Controls</h4>
<p>
Publishes to: <code>/cmd_vel</code>. Virtual Joystick is provided for controls. It offers a one-input device, that sends a vector of movement to the robot, by publishing it on command velocity topic
</p>

<h4>📷 Video Streaming</h4>
<p>
Publishes to: <code>/video_streaming_topic</code>. Video streaming is set to low resolution for faster viewing speed. Doesn't consume data while a user is not subscribed to the topic
</p>

<h4>📈 Sensors</h4>
<p>
Publishes to: <code>/sensors_topic</code>. Monitor sensors data from the robot. The most useful ones are network traffic while video streaming and CPU usage of the system
</p>

<h4>💾 Data Saving</h4>
<p>
Subscribes to: 

<ul>
    <li><code>/zed2/zed_node/left/image_rect_color</code></li>
    <li><code>/zed2/zed_node/right/image_rect_color"</code></li>
    <li><code>/zed2/zed_node/depth/depth_registered</code></li>
    <li><code>/zed2/zed_node/imu/data</code></li>
    <li><code>/rslidar_points</code></li>
    <li><code>/tf</code></li>
    <li><code>/gps</code></li>
</ul>

Save and analyze collected data for later. Size of camera images depends on set resolution of the zed wrapper
</p>

<h4>🤖 AI</h4>
<p>
Publishes to: <code>/ai_streaming</code>. AI takes an image from the zed wrapper, processes it, adds a bounding box with probability of the object and publishes it to the ai topic
</p>

---

## How It Works

<div align="center">
     <img src="https://github.com/emir-masinovic/RoboVoyager/assets/80472569/50c5f52a-71c6-4d77-8b4b-c0505a34d589" alt="#">
     <br>
     <br>
</div>

The browser's web page is the frontend, Rosbridge is the middleware, and ROS is the backend. In our project, Rosbridge and ROS are running on the same robot, which gives us better latency and we don't have to specify the connection from Rosbridge to ROS. The production build of the website is served from robot's computer as well, which means that devices that want to access the site have to be on the same LAN as the robot is.

ROS nodes represent the features. They have to communicate with each other and ROS to take data, process it, and publish it back to the system. The user subscribes/unsubscribes to these nodes from the website interface by click on their respective buttons.


> NOTE: It's possible to decouple these parts entirely and enable access of the client to the robot via the internet, but it requires buying a public IP from the ISP and working on network configuration, which was outside of the scope of the project. For more information visit: https://journals.sagepub.com/doi/10.1177/1729881417703355

---

## How To Use

#### Backend Side

This project runs on <a href="http://wiki.ros.org/noetic">ROS 1<img src="http://wiki.ros.org/noetic?action=AttachFile&do=get&target=noetic.png" alt="ros" width="20" height="20"/></a>, also known as ROS Noetic Ninjemys. The target OS for ROS is Ubuntu v20.04, although other distributions and operating systems are exprimentally supported. We recommend using Ubuntu and installing ROS with:

```bash
# Single line installation:
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

> NOTE: during installation, you will be prompted which installation to choose:
>
> - Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D > simulators and 2D/3D perception packages
> - Desktop Install: Everything in ROS-Base plus tools like rqt and rviz
> - ROS-Base: (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools

```bash
# After ROS is installed, check if it is sourced in the bashrc
gedit ~/.bashrc # Use any text editor like gedit or nano
# Scroll down until you see: source /opt/ros/noetic/setup.bash
# If it isn't there, close the editor and run:
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

After installing ROS, download RoboVoyager repository to your disk drive. We recommend putting it in the home directory for shorter navigation:


```bash
# Clone the repo
git clone https://github.com/emir-masinovic/RoboVoyager.git

# Navigate to it (home directory example)
cd ~/RoboVoyager

# Rosbridge server which will connect ROS with the browser interface
sudo apt install ros-noetic-rosbridge-server

# For AI and Video Streaming features, Open Computer Vision library
sudo apt install opencv

# For data collection, several input devices are used, like Zed 2 camera, Lidar, and GPS
sudo apt install serial # GPS
sudo apt install ros-noetic-nmea-msgs # GPS
sudo apt install ros-noetic-pointcloud-to-laserscan # Lidar
# For Zed 2 camera
sudo apt install ros-noetic-rviz-imu-plugin
sudo apt install ros-noetic-rtabmap
sudo apt install ros-noetic-rtabmap-ros
sudo apt install ros-noetic-depthimage-to-laserscan
cd /src
git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
cd ..

# Finally, compile everything
catkin_make
source devel/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release

# You can source the project workspace in bashrc for all terminal sessions (recommended)
echo "source ~/RoboVoyager/devel/setup.bash" >> ~/.bashrc
```

A list of [ROS commands](ROS_COMMANDS.md) is provided for running, testing nodes for development purposes

<br>


#### Frontend side

Nodejs and Sveltekit are used to make the website interface. RoboVoyager doesn't use styling libraries like Tailwind, only CSS.

Node version used at the time of development: v20.10.0

```bash

# Install Nodejs
sudo apt install nodejs

# Confirm installation
sudo nodejs -v

# Navigate
cd ~/RoboVoyager/frontend/sveltekit

# Install package.json dependencies
npm install

# Development mode on a local machine
npm run dev

# Development mode over LAN (for mobile testing)
npm run lan

# Production build
npm run build

# View the production build
npm run preview
```

---

## Related

These projects are from other groups that are working on the robot:


<ul>
    <li><a href="https://tuanacetinkaya.github.io/Moborobot/ ">Moborobo-Project</a></li>
    <li><a href="https://velicano.github.io/webpage/">Moborobo_GMapping</a></li>
</ul>

---

## License

This project is licensed under the [MIT License](http://opensource.org/licenses/MIT).

---

<p align="center">      <a href="https://www.linux.org/" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/linux/linux-original.svg" alt="linux" width="40" height="40"/></a>      <a href="https://www.python.org" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/python/python-original.svg" alt="python" width="40" height="40"/></a>       <a href="https://developer.mozilla.org/en-US/docs/Web/JavaScript" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/javascript/javascript-original.svg" alt="javascript" width="40" height="40"/></a>      <a href="https://www.w3.org/html/" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/html5/html5-original-wordmark.svg" alt="html5" width="40" height="40"/></a>       <a href="https://www.w3schools.com/css/" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/css3/css3-original-wordmark.svg" alt="css3" width="40" height="40"/></a>        <a href="https://svelte.dev" target="_blank" rel="noreferrer"> <img src="https://upload.wikimedia.org/wikipedia/commons/1/1b/Svelte_Logo.svg" alt="svelte" width="40" height="40"/></a>     <a href="https://opencv.org/" target="_blank" rel="noreferrer"> <img src="https://www.vectorlogo.zone/logos/opencv/opencv-icon.svg" alt="opencv" width="40" height="40"/></a>       <a href="https://pytorch.org/" target="_blank" rel="noreferrer"><img src="https://www.vectorlogo.zone/logos/pytorch/pytorch-icon.svg" alt="pytorch" width="40" height="40"/></a> </p>
