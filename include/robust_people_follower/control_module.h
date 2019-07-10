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


class ControlModule
{
public:
    ControlModule();

    // getters
    inline const std::shared_ptr<std::deque<geometry_msgs::PointStamped>> waypoints() const { return m_waypoint_list; }

    void addNewWaypoint(const geometry_msgs::Pose& t_target_pose, int t_times_per_second);
    geometry_msgs::Twist velocityCommand(StatusModule::Status& t_status,
                                         double t_angle,
                                         const geometry_msgs::Pose& t_pose,
                                         const Person& t_target);
    geometry_msgs::Twist velocityCommand(double t_angle, const geometry_msgs::Pose& t_pose,
                                         double t_x, double t_y);


private:
    double m_current_linear{};
    double m_current_angular{};
    std::shared_ptr<std::deque<geometry_msgs::PointStamped>> m_waypoint_list{};
    ros::Time m_last_waypoint_time{};
};


#endif //ROBUST_PEOPLE_FOLLOWER_CONTROL_MODULE_H
