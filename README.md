# A ROS package for the robust following of a person

## Description
This ROS node enables a robot equipped with the Orbbec Astra Pro RGB-D camera to follow a human and to re-identify the
target by applying a extrapolation of the target's last known position based on the CTRA motion model.

## Prerequisites
- Laptop running ROS Kinetic on Ubuntu 16.04 LTS
- Orbbec Astra Pro camera
- Turtlebot 2


## Installation

### Turtlebot ROS packages
In order for the Turtlebot 2 to work with ROS, a few packages have to be installed first.
```
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs
```

### CMake
You maybe already have CMake on your machine. If you don't have it, you need to download it from their homepage at
https://cmake.org/download/. After downloading, create a new directory and paste the downloaded file in this
directory and unzip it.
```
mkdir programs
cd programs/cmake-<your_version>
```
To install CMake run the following command (this can take a few minutes):
```
sudo ./bootstrap && make && make install
```

### OpenNI
Download the Orbbec OpenNI SDK from the following site: https://orbbec3d.com/develop/. Copy this zip file to the
programs directory you've made with the installation of CMake. Unzip the file in this directory. Go to:
```
cd programs/OpenNI_<your_version>/Linux
```
Here unzip the `OpenNI-Linux-x64-2.3` folder and run the following commands:
```
cd OpenNI-Linux-x64-2.3
chmod +x install.sh
sudo ./install.sh
```
Plug in your camera and run the command:
```
source OpenNIDevEnvironment
```
Before you can test visual samples, you need the `freeglut3` header and libraries, so install them using the following
commands:
```
sudo apt install build-essential freeglut3 freeglut3-dev
```
Since now `freeglut3` is installed, build a sample and try if it works:
```
cd Samples/SimpleViewer
make
cd Bin/x64-Release
./SimpleViewer
```

### Orbbec Astra SDK
Download the Orbbec Astra SDK zip file for your machine from the the Orbbec site: https://orbbec3d.com/develop/.
Copy this zip file to the directory `/programs` which you have made with the installation of CMake and unzip the file.
Rename the unzipped folder to `Astra_SDK` for easier navigation through the folders. Go to and run:
```
cd programs/Astra_SDK/install
chmod +x install.sh
sudo ./install.sh
```
To test if the Astra SDK is installed properly, you need to have the SFML libraries. (more information can be found at
their homepage at https://www.sfml-dev.org/):
```
sudo apt install libsfml-dev
```
Test the software with the following commands:
```
cd ..
cd bin
./SimpleBodyViewer-SFML
```
If the program is installed correctly, a new window pops up which recognizes a human and draws a skeleton in the picture
as shown below.

### astra_body_msgs
In order to install the message type for the `astra_body_tracker` package, use the following commands:
```
cd <your_catkin_workspace>/src
git clone https://github.com/shinselrobots/body_tracker_msgs.git
cd .. && catkin_make
```

### astra_body_tracker
Next, the `astra_body_tracker` package has to be installed. To clone the repository, use the following commands:
```
cd <your_catkin_workspace>/src
git clone https://github.com/KrisPiters/astra_body_tracker.git
```
Before running `catkin_make`, make sure the path to the Astra SDK in the `CMakeLists.txt` is updated to the path to your
Astra_SDK. To do that, open the `CMakeLists.txt` and change the paths at line 19 - 22 to the path to your Astra_SDK
directory:
```
cd astra_body_tracker
nano CMakeLists.txt
```

When changed, build the package:
```
cd ../.. && catkin_make
```

### This package
For the last part, now you need to install this package. Go to the source of your catkin workspace and clone this repository:
```
cd <your_catkin_workspace>/src
git clone https://github.com/sijanz/robust_people_follower
cd .. && catkin_make
```


### Enable networking
Even though this ROS node works perfectly fine when running locally on the laptop connected to the Turtlebot 2, in order
to view markers in RViz that are published by this node, you have to enable networking on the Turtlebot.

To set up the network configuration, follow this tutorial: http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration


## Usage
If the camera and the Kobuki base are both connected to your laptop running ROS, execute the following command to start
the Turtlebot:
```
roslaunch turtlebot_bringup minimal.launch
```

After that, use the following command to start this node:
```
roslaunch robust_people_follower robust_people_follower.launch
```

After startup, the robot automatically is in a passive WAITING mode. In order to select yourself as the target,
perform the following gesture: Close both of your hands to fists and hold them above your shoulder. Hold this gesture
for at least 3 seconds.  

![Caption for the picture.](images/target_selection.png?raw=true)

The robot is now following you. If the robot should loose the line of sight to you, it tries to re-identify you. If the
re-identification is successful, the robot keep following you. If the robot fails to re-identify you, its mode is set to
WAITING. In order de-select yourself as the target, perform the above mentioned gesture again during following.

This node publishes markers that can be viewed in RViz to get a visualization of the robot's surroundings. After networking
on the Turtlebot 2 is configured, open RViz using the following command:
```
roslaunch turtlebot_rviz_launchers view_robot.launch
```
Then select the ROS topic `robust_people_follower/markers` to display the markers published by this node.

![Caption for the picture.](images/rviz.png?raw=true)

Markers published include:
- position of tracked persons (yellow marker in form of a human)
- position of the current target (green marker in form of a human)
- velocity vector of tracked persons (red arrow)
- waypoints of the robot (red column)
- replicated path (blue line in between waypoint markers)
- last known position of the target (solid blue marker in form of a person)
- extrapolated position of the target after target loss (semi-transparent blue person marker)
- probability radius in which the target is re-identified (semi-transparent blue circle)