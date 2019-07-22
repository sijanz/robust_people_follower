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


#ifndef ROBUST_PEOPLE_FOLLOWER_CONTROL_MODULE_H
#define ROBUST_PEOPLE_FOLLOWER_CONTROL_MODULE_H


#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include "robust_people_follower/status_module.h"


/**
 * @brief The Control Module creates velocity commands to control the robot. In order to replicates the target's path,
 * it also holds and manages a list of waypoints.
 */
class ControlModule
{
public:

    /*
     * ********** SPECIAL METHODS **********
     */

    /**
     * @brief Constructor. Allocates the list for waypoints on the heap and initializes all the members.
     */
    ControlModule();


    /*
     * ********** GETTERS **********
     */

    /**
     * @brief Getter for the waypoint list.
     *
     * @return a pointer to the waypoint list
     */
    inline const std::shared_ptr<std::deque<geometry_msgs::PointStamped>> waypoints() const { return m_waypoint_list; }


    /*
     * ********** STANDARD METHODS **********
     */

    /**
     * @brief Adds a waypoint where the target is located at the moment.
     *
     * @param t_target_pose the pose of the current target
     * @param t_times_per_second how many times per second a waypoint is to be placed
     */
    void addNewWaypoint(const geometry_msgs::PoseStamped& t_target_pose, int t_times_per_second);


    /**
     * @brief Creates a velocity command that can be published. If the target is closer than 1 meter, a command to drive
     * the robot backwards while keeping the target centered is created. If the target is between 1 meter and the
     * following threshold, a command to only keep the target centered is created. If the target is further away than
     * the following threshold, the target's path is replicated by driving to the next waypoint in the list. If current
     * status is LOS_LOST and the waypoint list is empty, the status is set to SEARCHING.
     *
     * @param t_status reference to the status of the robot; gets changed to SEARCHING if waypoint list is empty
     * @param t_pose pose of the robot
     * @param t_target reference to the current target; needed to get its position
     * @param t_follow_threshold the distance at which the target is being followed
     * @return the velocity command to be published in ROS; includes linear and angular velocity information
     */
    geometry_msgs::Twist velocityCommand(StatusModule::Status& t_status, const geometry_msgs::PoseStamped& t_pose,
                                         const Person& t_target, double t_follow_threshold);


    /**
     * @brief Creates a velocity command that can be published. This method is used to keep a given point centered in
     * the robot's field of view.
     *
     * @param t_pose the pose of the robot
     * @param t_point the point to look at
     * @return the velocity command to be published in ROS; includes linear and angular velocity information
     */
    static geometry_msgs::Twist velocityCommand(const geometry_msgs::PoseStamped& t_pose,
                                                const geometry_msgs::Point32& t_point);


private:

    /** @brief The current linear velocity component of the robot. */
    double m_current_linear{};

    /** @brief The current angular velocity component of the robot. */
    double m_current_angular{};

    /** @brief A pointer to the list of waypoints located on the heap. */
    std::shared_ptr<std::deque<geometry_msgs::PointStamped>> m_waypoint_list{};

    /** @brief The time the last waypoint is set. */
    ros::Time m_last_waypoint_time{};
};


#endif //ROBUST_PEOPLE_FOLLOWER_CONTROL_MODULE_H
