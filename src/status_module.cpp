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


#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>

#include "robust_people_follower/status_module.h"


void StatusModule::calculateAngle()
{
    auto q{tf::Quaternion{m_pose.orientation.x, m_pose.orientation.y, m_pose.orientation.z, m_pose.orientation.w}};
    auto m{tf::Matrix3x3{q}};

    auto roll{0.0}, pitch{0.0}, theta{0.0};
    m.getRPY(roll, pitch, theta);

    m_angle = theta;
}


void StatusModule::calculateVelocity(const double t_frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / t_frequency);
}


void StatusModule::printInfo() const
{
    auto status_string{""};
    switch (m_status) {
        case WAITING:
            status_string = "WAITING";
            break;
        case FOLLOWING:
            status_string = "FOLLOWING";
            break;
        case LOS_LOST:
            status_string = "LOS_LOST";
            break;
        default:
            status_string = "SEARCHING";
            break;
    }

    ROS_INFO("Robot information:");
    ROS_INFO("  status: [%s]", status_string);
    ROS_INFO("  velocity: %f", m_velocity);
    ROS_INFO("  position:");
    ROS_INFO("    x: %f", m_pose.position.x);
    ROS_INFO("    y: %f", m_pose.position.y);
    ROS_INFO("  theta: %f", m_angle);
}


void StatusModule::processOdometryData(const geometry_msgs::Pose& t_pose, const double t_frequency)
{
    m_pose = t_pose;
    calculateAngle();
    calculateVelocity(t_frequency);
    updatePose();
}
