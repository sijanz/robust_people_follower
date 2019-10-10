# A ROS package for robust following of a person

## Description

## Prerequisites

## Installation

## Usage
When starting the node, the robot automatically is in a passive WAITING mode. In order to select yourself as the target,
perform the following gesture: Close both of your hands to fists and hold them above your shoulder. Hold this gesture for
at least 3 seconds.  
![Caption for the picture.](images/target_selection.png?raw=true)

The robot is now following you. If the robot should loose the line of sight to you, it tries to re-identify you. If the
re-identification is successful, the robot keep following you. If the robot fails to re-identify you, its mode is set to
WAITING. In order de-select yourself as the target, perform the above mentioned gesture again during following.