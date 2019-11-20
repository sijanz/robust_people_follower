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


#ifndef ROBUST_PEOPLE_FOLLOWER_ROBUST_PEOPLE_FOLLOWER_NODE_H
#define ROBUST_PEOPLE_FOLLOWER_ROBUST_PEOPLE_FOLLOWER_NODE_H


#include <deque>

#include <ros/ros.h>
#include <kobuki_msgs/BumperEvent.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include "status_module.h"
#include "tracking_module.h"
#include "control_module.h"
#include "recovery_module.h"


/**
 * @brief This class represents the ROS node. It defines the subscribers along with their callback functions, and
 * publishers. In addition to the ROS interface, important constants like the loop function are defined here. The node's
 * main loop is also defined in this class.
 */
class RobustPeopleFollower
{
public:

    /*
     * ********** SPECIAL METHODS **********
     */

    /**
     * @brief Constructor for a RobustPeopleFollower object. Allocates memory on the heap for the robot's and target'
     * path, sets the name of the node. Initializes the subscribers that subscribe to the "/odom"- and
     * "/body_tracker/skeleton"- topic. Initializes the publishers that publish to the "/mobile_base/commands/velocity"-,
     * "robust_people_follower/robot_path"-, "robust_people_follower/target_path"- and "robust_people_follower/markers"-
     * topic.
     *
     * @param t_name the name of the node as indicated in ROS
     */
    explicit RobustPeopleFollower(const std::string& t_name);


    /**
     * @brief Destructor for a RobustPeopleFollower object. Shuts down the subscribers and publishers and prints out a
     * message that indicates the shutdown.
     */
    ~RobustPeopleFollower();


    /*
     * ********** CALLBACK FUNCTIONS **********
     */

    void bumperCallback(const kobuki_msgs::BumperEventConstPtr& msg);

    /**
     * @brief Callback function for odometry data. Stores data from messages of the "/odom"-topic and assigns them to
     * the Robot-member object. The odometry data consists of the robots position and orientation, known as a "pose".
     * After storing the data received from the message, the Euler angle of the robot and the robot's position is
     * calculated.
     *
     * @param msg a pointer to the message received from the "/odom"-topic
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);


    /**
     * @brief Callback function for skeleton data. Stores data from messages of the "body_tracker/skeleton"-topic and
     * assigns them to a Person-object. Look at the documentation for "body_tracker_msgs" to get an overview of skeleton
     * information. If a hand gesture count of 2 is registered for more than 3 seconds, the corresponding person is set
     * as target. Person objects are stored in list owned by Robot.
     *
     * @param msg a pointer to the message received from the "body_tracker/skeleton"-topic
     */
    void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg);


    /*
     * ********** MAIN LOOP **********
     */

    /**
     * @brief Represents the entry point and the main loop of the node. For each iteration, the last target's position
     * is first saved. After that, the callback functions are processed and the data from the subscribed topics is
     * stored accordingly. Next, the robot's status gets checked and updated. If the target is lost, the recovery
     * function is called and attempts to re-identify the target. To help getting an overview of the scenario,
     * estimation markers are published. After that, waypoints are added if the target is above a following threshold.
     * In order to finish a loop iteration, a velocity command is published and the state variables of both the robot
     * and the tracked persons are updated.
     */
    void runLoop();


private:

    /*
     * ********** CONSTANTS **********
     */

    /** @brief The frequency in which the main loop is running. */
    static constexpr double LOOP_FREQUENCY{10.0};

    /** @brief The distance threshold after that the target is being followed (in mm). */
    static constexpr double FOLLOW_THRESHOLD{1800};

    /** @brief The time period to re-identify the target. */
    static constexpr int SEARCH_TIMEOUT{10};

    /** @brief The factor of the length of a person's vector. */
    static constexpr double VECTOR_LENGTH_FACTOR{1.5};


    /*
     * ********** STANDARD MEMBERS **********
     */

    /** @brief The name of the node as represented in ROS. */
    std::string m_name{};

    /** @brief The ROS node handle. Is used to subscribe and publish to ROS topics. */
    ros::NodeHandle m_nh{};

    /** @brief Is set to true if the robot runs into an obstacle. */
    bool m_emergency_stop{};


    /*
     * ********** ROS SUBSCRIBERS **********
     */

    /** @brief Subscribes to the "/odom"-topic at a rate of 10 times per second. */
    ros::Subscriber m_odom_sub{};

    /** @brief Subscribes to the "body_tracker/skeleton"-topic at a rate of 10 times per second. */
    ros::Subscriber m_skeleton_sub{};

    ros::Subscriber m_bumper_sub{};


    /*
     * ********** ROS PUBLISHERS **********
     */

    /** @brief Publishes a velocity command to the "/mobile_base/commands/velocity"-topic at a rate of 1000 times per
     * second. */
    ros::Publisher m_velocity_command_pub{};

    /** @brief Publishes the tracked persons' vector and position to the "robust_people_follower/markers"-topic at a
     * rate of 10 times per second. */
    ros::Publisher m_visualization_pub{};


    /*
     * ********** MODULES **********
     */

    /** @brief Instance of the Status Module. Keeps track of the robot's status as well as its pose and velocity. See
     * status_module.h for detailed documentation. */
    StatusModule m_status_module{};

    /** @brief Instance of the Tracking Module. Manages the tracked persons and specifically information about the
     * target. See tracking_module.h for detailed documentation. */
    TrackingModule m_tracking_module{};

    /** @brief Instance of the Control Module. Calculates the robot's velocity messages. See control_module.h for
     * detailed documentation. */
    ControlModule m_control_module{};

    /** @brief Instance of the Recovery Module. Predicts the target's position if the line of sight is lost and
     * re-identifies the target. See recovery_module.h for detailed documentation. */
    RecoveryModule m_recovery_module{};


    /*
     * ********** HELPER METHODS TO KEEP THE MAIN LOOP CLEAN **********
     */

    /**
     * @brief Processes the ROS callbacks and sets a new target if available.
     */
    void processCallbacks();


    /**
     * @brief Prints out information about the robot's and the target's status as well as information about tracked
     * persons.
     */
    void debugPrintout() const;


    /**
     * @brief Adds a new waypoint if the target is above the following threshold and then follows the target in
     * dependence of the distance.
     */
    void followTarget();


    /**
     * @brief Predicts the target's position, moves the robot to the last known position of the target and tries to
     * re-identify the target.
     */
    void searchForTarget();


    /**
     * @brief Publishes the person's position and vector markers using the ROS node handle and the ROS visualization
     * publisher.
     */
    void publishPersonMarkers() const;


    /**
     * @brief Publishes waypoint markers using the ROS node handle and the ROS visualization publisher.
     */
    void publishWaypoints() const;


    /*
     * ********** RVIZ MARKERS **********
     */

    /**
     * @brief Returns a visualization marker that represents the position the target was last seen.
     *
     * @return a marker to the target's last position
     */
    visualization_msgs::Marker lastPositionMarker() const;


    /**
     * @brief Returns a visualization marker that represents the current position of the predicted target.
     *
     * @return a marker to the target's predicted position
     */
    visualization_msgs::Marker predictedPositionMarker() const;


    /**
     * @brief Returns a marker of the area in which the target is assumed to be.
     *
     * @return a marker to the target's assumed area
     */
    visualization_msgs::Marker predictionRadiusMarker() const;
};


#endif //ROBUST_PEOPLE_FOLLOWER_ROBUST_PEOPLE_FOLLOWER_NODE_H
