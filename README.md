# A ROS package for robust following of a person

## Description


## Prerequisites
- Laptop running ROS Kinetic on Ubuntu 16.04 or higher
- Orbbec Astra Pro camera
- Turtlebot 2


## Installation
### CMake
You maybe already have CMake on your machine. If you don't have it, you need to dowload it from their webpage: https://cmake.org/download/. When the dowload is finished, make a new directory and paste the downloaded file in this directory and unzip it.
```
cd
mkdir programs
cd programs/cmake-<your_version>
```
To install CMake run the following command (this can take a few minutes)
```
sudo ./bootstrap && make && make install
```

### OpenNI
Download the Orbbec OpenNI SDK from the following site: https://orbbec3d.com/develop/. Copy this zip file to the programs directory you've made with the installation of CMake. Unzip the file in this directry. Go to:
```
cd programs/OpenNI_<your_version>/Linux
```
Here unzip the OpenNI-Linux-x64-2.3 folder and run the following commands:
```
cd OpenNI-Linux-x64-2.3
chmod +x install.sh
sudo ./install.sh
```
Plug your camera and run the command:
```
source OpenNIDevEnvironment
```
Before you can test visual samples, you will need freeglut3 header and libaries, please install:
```
sudo apt-get install build-essential freeglut3 freeglut3-dev
```
Now freeglut3 is install, build a sample and try if it works
```
cd Samples/SimpleViewer
make
cd Bin/x64-Release
./SimpleViewer
```
If the program is correctly installed a new window opens with a yellow depth video stream as shown below.


### Orbbec Astra SDK
Download the Orbbec astra SDK zip file for your machine from the the Orbbec site: https://orbbec3d.com/develop/. Copy this zip file to the directeroy /programs which you have made with the installation of CMake and unzip the file. Rename the unzipped folder to Astra_SDK for easier navigation through the folders. Go to and run:
```
cd programs/Astra_SDK/install
chmod +x install.sh
sudo ./install.sh
```
To test if the Astra SDK is installed properly, you need to have the SFML libraries. (more information can be found at their webpage: https://www.sfml-dev.org/)
```
sudo apt-get install libsfml-dev
```
Test the software with the following commands:
```
cd ..
cd bin
./SimpleBodyViewer-SFML
```
If the program is installed correctly, a new window pops up wich recognizes a human and draws a skeleton in the picture as shown below.

### astra_body_msgs
The next step is to clone the package with the message type for the astra_body_tracker from this github repository: https://github.com/shinselrobots/body_tracker_msgs. In the terminal go to the source of your /catkin_ws and clone and build this package:
```
cd catkin_ws/src
git clone https://github.com/shinselrobots/body_tracker_msgs.git
cd ..
catkin_make
```

### astra_body_tracker
First step is to clone the astra_body_tracker from this github repository: https://github.com/KrisPiters/astra_body_tracker. In the terminal go to the source of your catkin_ws and clone and build this package:
```
cd catkin_ws/src
git clone https://github.com/KrisPiters/astra_body_tracker.git
```
Before running catkin_make, make sure the path to the Astra_SDK in the CMakeLists.txt is updated to the path to your Astra_SDK. Herefor, open the CMakeLists.txt
```
gedit CMakeLists.txt
```
Change the paths at line 19 - 22 to the path to your Astra_SDK directory. When changed, build the package:
```
cd ..
catkin_make
```

### This package
For the last part, now you need to install this package. Go to the source of the catkin_ws and clone this repository:
```
cd catkin_ws/src
git clone https://github.com/sijanz/robust_people_follower
catkin_make
```

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
perform the following gesture: Close both of your hands to fists and hold them above your shoulder. Hold this gesture for
at least 3 seconds.  
![Caption for the picture.](images/target_selection.png?raw=true)

The robot is now following you. If the robot should loose the line of sight to you, it tries to re-identify you. If the
re-identification is successful, the robot keep following you. If the robot fails to re-identify you, its mode is set to
WAITING. In order de-select yourself as the target, perform the above mentioned gesture again during following.