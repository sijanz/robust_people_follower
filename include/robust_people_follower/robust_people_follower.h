/*********************************************************************
* Software License Agreement (BSD 3-Clause License)
*
*  Copyright (c) 2019, Simon Janzon
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#ifndef ROBUST_PEOPLE_FOLLOWER_ROBUST_PEOPLE_FOLLOWER_H
#define ROBUST_PEOPLE_FOLLOWER_ROBUST_PEOPLE_FOLLOWER_H


#include <deque>

#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include "robot.h"


class RobustPeopleFollower
{
public:
    explicit RobustPeopleFollower(const std::string& t_name);
    ~RobustPeopleFollower();
    void runLoop();
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg);

private:

    // name of the node
    std::string m_name{};

    // ROS node handle
    ros::NodeHandle m_nh{};

    // subscribers
    ros::Subscriber m_odom_sub{};
    ros::Subscriber m_skeleton_sub{};

    // publishers
    ros::Publisher m_velocity_command_pub{};
    ros::Publisher m_robot_path_pub{};
    ros::Publisher m_target_path_pub{};
    ros::Publisher m_visualization_pub{};

    // frequency of the main loop
    const double LOOP_FREQUENCY{10.0};

    // factor for person vector length
    const double VECTOR_LENGTH_FACTOR{1.0};

    const double FOLLOW_THRESHOLD{1800};

    // instance of the turtlebot robot
    Robot m_robot{};

    // instance of the target to follow
    Person m_target{};

    // list of persons in the frame
    std::unique_ptr<std::vector<Person>> m_tracked_persons{};

    // stores the path of the robot
    std::unique_ptr<nav_msgs::Path> m_robot_path{};

    // stores the path of the target
    std::unique_ptr<nav_msgs::Path> m_target_path{};

    // sequence number for robot path
    uint32_t m_seq_robot{};

    // sequence number for target path
    uint32_t m_seq_target{};

    // helper methods to keep the main loop tidy
    void debugPrintout();
    void publishPaths();
    void publishPersonMarkers() const;
    void publishWaypoints() const;
    void managePersonList();
    void updateTargetPath();
    visualization_msgs::Marker estimationMarker() const;
    visualization_msgs::Marker estimationVector() const;
};


#endif //ROBUST_PEOPLE_FOLLOWER_ROBUST_PEOPLE_FOLLOWER_H
