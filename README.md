<!-- ![RoboVoyager2](https://github.com/emir-masinovic/RoboVoyager/assets/80472569/c17de3c4-34ff-4b23-a62f-9f810c1693a0) -->

<div align="center">
     <a href="#">
        <img src="https://github.com/emir-masinovic/RoboVoyager/assets/80472569/c17de3c4-34ff-4b23-a62f-9f810c1693a0" alt="#">
     </a>
     <br>
     <br>
     <p>Website interface and a group of Robot Operating System (ROS) nodes that let you control a robot and monitor its sensors.</p>
     <img alt="Static Badge" src="https://img.shields.io/badge/license-MIT-blue">
</div>

<!-- <br> -->

<p align="center">
  <a href="#key-features">Key Features</a>&nbsp;&nbsp;
  <a href="#how-it-works">How It Works</a>&nbsp;&nbsp;
  <a href="#how-to-use">How To Use</a>&nbsp;&nbsp;
  <!-- <a href="#credits">Credits</a>&nbsp;&nbsp; -->
  <a href="#related">Related</a>&nbsp;&nbsp;
  <a href="#license">License</a>
</p>

---

## Key Features

<!-- - Controls: virtual joystick that updates <code>cmd_vel</code> topic and moves the robot
- Video Streaming: doesn't consume data while not subscribed to the topic
- Sensor Monitoring: extract basic sensors from the computer, as well as GPS coordinates
- Offline Maps and Orientation: find the robot's position even while offine
- [KaTeX](https://khan.github.io/KaTeX/)
- Data Saving: collects data from Lidar, Zed2, GPS
- Rosbridge Websockets: full duplex for data communication between website and ROS
- Cross platform -->

<h4>Controls</h4>
<p>
<svg fill="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg" width="20" height="20" style="vertical-align: middle;"><g id="SVGRepo_bgCarrier" stroke-width="0"></g><g id="SVGRepo_tracerCarrier" stroke-linecap="round" stroke-linejoin="round"></g><g id="SVGRepo_iconCarrier" ><path d="M20 7h-3V4c0-1.103-.897-2-2-2H9c-1.103 0-2 .897-2 2v3H4c-1.103 0-2 .897-2 2v6c0 1.103.897 2 2 2h3v3c0 1.103.897 2 2 2h6c1.103 0 2-.897 2-2v-3h3c1.103 0 2-.897 2-2V9c0-1.103-.897-2-2-2zm0 8h-5v4h.001v1H9v-5H4V9h5V4h6v5h5v6z" ></path><path d="M8 14v-4l-3 2zm8 0 3-2-3-2zm-6-6h4l-2-3zm2 11 2-3h-4z"></path><circle cx="12" cy="12" r="2"></circle></g></svg>
Topic: <code>/cmd_vel</code>. Virtual Joystick is provided for controls. It offers a one-input device, that sends a vector of movement directly to the robot (no need for a node), by publishing it on command velocity topic
</p>

<h4>Video Streaming</h4>
<p>
<svg 
    xmlns="http://www.w3.org/2000/svg" 
    fill="currentColor" 
    class="bi bi-camera-video" 
    viewBox="0 0 16 16" width="20" height="20" style="vertical-align: middle;"
    ><path fill-rule="evenodd" d="M0 5a2 2 0 0 1 2-2h7.5a2 2 0 0 1 1.983 1.738l3.11-1.382A1 1 0 0 1 16 4.269v7.462a1 1 0 0 1-1.406.913l-3.111-1.382A2 2 0 0 1 9.5 13H2a2 2 0 0 1-2-2zm11.5 5.175 3.5 1.556V4.269l-3.5 1.556zM2 4a1 1 0 0 0-1 1v6a1 1 0 0 0 1 1h7.5a1 1 0 0 0 1-1V5a1 1 0 0 0-1-1z" />
</svg>
Topic: <code>/video_streaming_topic</code>. Video streaming is set to low resolution for faster viewing speed. While websockets allow fast transportation, real time communicaton (RTC) protocol is probably better for image and media related things (in case you are looking for a better quality video). Doesn't consume data while not subscribed to the topic 
</p>

<h4>Sensors</h4>
<p>
<svg
    xmlns="http://www.w3.org/2000/svg"
    fill="currentColor"
    class="bi bi-card-list"
    viewBox="0 0 16 16"
    width="20" height="20" style="vertical-align: middle;"
><path
    d="M14.5 3a.5.5 0 0 1 .5.5v9a.5.5 0 0 1-.5.5h-13a.5.5 0 0 1-.5-.5v-9a.5.5 0 0 1 .5-.5zm-13-1A1.5 1.5 0 0 0 0 3.5v9A1.5 1.5 0 0 0 1.5 14h13a1.5 1.5 0 0 0 1.5-1.5v-9A1.5 1.5 0 0 0 14.5 2z"
/><path
    d="M5 8a.5.5 0 0 1 .5-.5h7a.5.5 0 0 1 0 1h-7A.5.5 0 0 1 5 8m0-2.5a.5.5 0 0 1 .5-.5h7a.5.5 0 0 1 0 1h-7a.5.5 0 0 1-.5-.5m0 5a.5.5 0 0 1 .5-.5h7a.5.5 0 0 1 0 1h-7a.5.5 0 0 1-.5-.5m-1-5a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0M4 8a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0m0 2.5a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0"
/>
</svg>
Topic: <code>/sensor_topic</code>. Monitor sensor data from your computer
</p>

<h4>Map with GPS</h4>
<p>
<svg 
    viewBox="0 0 24 24" 
    fill="none" 
    xmlns="http://www.w3.org/2000/svg"
    width="20" height="20" style="vertical-align: middle;"><g id="SVGRepo_bgCarrier" stroke-width="0"></g><g id="SVGRepo_tracerCarrier" stroke-linecap="round" stroke-linejoin="round"></g><g id="SVGRepo_iconCarrier"><path d="M20 12C20 16.4183 16.4183 20 12 20C7.58172 20 4 16.4183 4 12C4 7.58172 7.58172 4 12 4C16.4183 4 20 7.58172 20 12Z" stroke="currentColor" stroke-width="1.5"></path><path d="M15 12C15 13.6569 13.6569 15 12 15C10.3431 15 9 13.6569 9 12C9 10.3431 10.3431 9 12 9C13.6569 9 15 10.3431 15 12Z" stroke="currentColor" stroke-width="1.5"></path><path d="M2 12L4 12" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"></path><path d="M20 12L22 12" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"></path><path d="M12 4V2" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"></path><path d="M12 22V20" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"></path></g>
</svg>
Topic: <code>/gps</code>. Maps work even offline. Full screen for a better view
</p>

<h4>Data Saving</h4>
<p>
<svg
    xmlns="http://www.w3.org/2000/svg"
    fill="currentColor"
    class="bi bi-pencil-square"
    viewBox="0 0 16 16"
    width="20" height="20" style="vertical-align: middle;"
    ><path
        d="M15.502 1.94a.5.5 0 0 1 0 .706L14.459 3.69l-2-2L13.502.646a.5.5 0 0 1 .707 0l1.293 1.293zm-1.75 2.456-2-2L4.939 9.21a.5.5 0 0 0-.121.196l-.805 2.414a.25.25 0 0 0 .316.316l2.414-.805a.5.5 0 0 0 .196-.12l6.813-6.814z"
    /><path
        fill-rule="evenodd"
        d="M1 13.5A1.5 1.5 0 0 0 2.5 15h11a1.5 1.5 0 0 0 1.5-1.5v-6a.5.5 0 0 0-1 0v6a.5.5 0 0 1-.5.5h-11a.5.5 0 0 1-.5-.5v-11a.5.5 0 0 1 .5-.5H9a.5.5 0 0 0 0-1H2.5A1.5 1.5 0 0 0 1 2.5z"/>
</svg>
Topic: <code>/multimedia_capture</code>. Save and analyze collected data for later
</p>

---

## How It Works

<div align="center">
     <a href="#">
        <img src="https://foxglove.dev/images/blog/using-rosbridge-with-ros1/hero.webp" style="background:aliceblue;" alt="#">
     </a>
     <br>
     <br>

</div>

Browser is the frontend (client), Rosbridge is the middleware, and ROS is the backend. In our project, Rosbridge and ROS are running on the same robot, which gives us better latency and we don't have to specify the connection from Rosbridge to ROS. The production build of the website is served from robot's computer (Jetson Orin in our case), which means that devices that want to access the site, will have to be connected on the same LAN as the robot is. It's possible to decouple these parts entirely and enable access of the client to the robot via the internet, but it requires buying a public IP from the ISP and working on network configuration, which was outside of the scope of the project. However, this is a good link to read.

https://journals.sagepub.com/doi/10.1177/1729881417703355

---

## How To Use

#### Backend Side

This project runs on <a href="http://wiki.ros.org/noetic" target="_blank" rel="noreferrer">ROS 1<img src="http://wiki.ros.org/noetic?action=AttachFile&do=get&target=noetic.png" alt="ros" width="20" height="20"/></a>, also known as ROS Noetic Ninjemys. The target OS for ROS is Ubuntu v20.04, altough other distributions and operating systems are exprimentally supported. We recommend using Ubuntu and installing ROS with:

```bash
# Single line installation:
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

> **NOTE**: during installation, you will be prompted which installation to choose:
>
> - Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D > simulators and 2D/3D perception packages
> - Desktop Install: Everything in ROS-Base plus tools like rqt and rviz
> - ROS-Base: (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools

After installing ROS, download RoboVoyager repository to your disk drive. We recommend putting it in the home directory for shorter navigation:

```bash
# Clone the repo
git clone https://github.com/emir-masinovic/RoboVoyager.git

# Navigate to it (home directory example)
cd ~/RoboVoyager

# Some dependencies have to installed
# Open Computer Vision library (used for AI and video streaming features)
sudo apt install opencv

# Rosbridge server connects ROS with the browser, website
sudo apt install ros-noetic-rosbridge-server

# Serial is used for GPS
sudo apt install serial

# Run catkin_make to generate devel and build folders
catkin_make

# If you can't run catkin_make, make sure ROS is sourced by running
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# You can source the project workspace in bashrc (recommended)
gedit ~/.bashrc # Use any text editor like gedit or nano
# Scroll down until you see: source /opt/ros/noetic/setup.bash
# Copy your workspace location (home dir example)
source ~/RoboVoyager/devel/setup.bash
# Now you can run ROS commands from the terminal regardless of sessions

# You can source the project in every new terminal tab (if you have multiple
# ROS projects and want to avoid potential conflicts)
# Navigate to RoboVoyager directory, and then
source devel/setup.bash
# Note: this will have to be repeated every time upon starting a new session
```

ROS commands are written **here**.

#### Frontend side

Nodejs and Sveltekit are used to make the website interface. RoboVoyager doesn't use styling libraries like Tailwind, only CSS.

Node version used at the time of development: v20.10.0

```bash

# Install Nodejs
sudo apt install nodejs

# Confirm installation
sudo nodejs -v

# Navigate to your workspace (home dir in example)
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

https://tuanacetinkaya.github.io/Moborobot/

---

## ©️ License

This project is licensed under the [MIT License](http://opensource.org/licenses/MIT).

---

## ❤️ Contributors

There is no contributor yet. Want to be the first ?

<!-- If you want to contribute to this project, please read the [**contribution guide**](https://github.com/antoinezanardi/werewolves-assistant-api-next/blob/master/CONTRIBUTING.md). -->

---

<p align="center">      <a href="https://www.linux.org/" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/linux/linux-original.svg" alt="linux" width="40" height="40"/></a>      <a href="https://www.python.org" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/python/python-original.svg" alt="python" width="40" height="40"/></a>       <a href="https://developer.mozilla.org/en-US/docs/Web/JavaScript" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/javascript/javascript-original.svg" alt="javascript" width="40" height="40"/></a>      <a href="https://www.w3.org/html/" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/html5/html5-original-wordmark.svg" alt="html5" width="40" height="40"/></a>       <a href="https://www.w3schools.com/css/" target="_blank" rel="noreferrer"><img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/css3/css3-original-wordmark.svg" alt="css3" width="40" height="40"/></a>        <a href="https://svelte.dev" target="_blank" rel="noreferrer"> <img src="https://upload.wikimedia.org/wikipedia/commons/1/1b/Svelte_Logo.svg" alt="svelte" width="40" height="40"/></a>     <a href="https://opencv.org/" target="_blank" rel="noreferrer"> <img src="https://www.vectorlogo.zone/logos/opencv/opencv-icon.svg" alt="opencv" width="40" height="40"/></a>       <a href="https://pytorch.org/" target="_blank" rel="noreferrer"><img src="https://www.vectorlogo.zone/logos/pytorch/pytorch-icon.svg" alt="pytorch" width="40" height="40"/></a> </p>
