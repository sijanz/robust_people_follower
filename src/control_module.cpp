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


#include <deque>

#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <robust_people_follower/person.h>

#include "robust_people_follower/control_module.h"


ControlModule::ControlModule() : m_current_linear{}, m_current_angular{}, m_last_waypoint_time{}
{
    m_waypoint_list = std::make_shared<std::deque<geometry_msgs::PointStamped>>();
}


void ControlModule::addNewWaypoint(const geometry_msgs::Pose& t_target_pose, const int t_times_per_second)
{
    if (m_waypoint_list->size() > 100)
        m_waypoint_list->pop_front();

    if (ros::Time::now() - m_last_waypoint_time > ros::Duration(0, (1000000000 / t_times_per_second))) {
        auto position{geometry_msgs::PointStamped{}};
        position.header.stamp = ros::Time::now();
        position.point.x = t_target_pose.position.x;
        position.point.y = t_target_pose.position.y;

        m_last_waypoint_time = ros::Time::now();
        m_waypoint_list->emplace_back(position);
    }
}


geometry_msgs::Twist ControlModule::velocityCommand(StatusModule::Status& t_status,
                                                    const double t_angle,
                                                    const geometry_msgs::Pose& t_pose,
                                                    const Person& t_target)
{

    // TODO: declare as member
    auto FOLLOW_THRESHOLD{1800.0};

    auto distance_to_target{t_target.distance()};

    // FIXME: ugly, only temporary fix
    if (distance_to_target == 0)
        distance_to_target = FOLLOW_THRESHOLD + 200;

    if (distance_to_target < FOLLOW_THRESHOLD)
        m_waypoint_list->clear();

    if (t_status == StatusModule::Status::LOS_LOST && m_waypoint_list->empty())
        t_status = StatusModule::Status::SEARCHING;

    auto speed{geometry_msgs::Twist{}};

    // if target is too near, move backwards
    if (distance_to_target < 1000) {

        speed.linear.x = 2 * (distance_to_target / 1000) - 2;

        if (t_target.yDeviation() < -50) {
            speed.angular.z = -0.0025 * std::abs(t_target.yDeviation());
        } else if (t_target.yDeviation() > 50) {
            speed.angular.z = 0.0025 * t_target.yDeviation();
        } else
            speed.angular.z = 0;

        // target is under threshold, only keep him centered
    } else if (distance_to_target > 1000 && distance_to_target < FOLLOW_THRESHOLD) {

        if (t_target.yDeviation() < -50) {
            speed.angular.z = -0.0025 * std::abs(t_target.yDeviation());
        } else if (t_target.yDeviation() > 50) {
            speed.angular.z = 0.0025 * t_target.yDeviation();
        } else
            speed.angular.z = 0;

        // target is above threshold, follow him using waypoints
    } else if (!m_waypoint_list->empty()) {

        auto current_goal{m_waypoint_list->at(0)};

        auto rotation{tf::Matrix3x3{
                cos(t_angle), -sin(t_angle), t_pose.position.x,
                sin(t_angle), cos(t_angle), t_pose.position.y,
                0.0, 0.0, 1.0
        }};

        auto global_vector{tf::Vector3{current_goal.point.x, current_goal.point.y, 1.0}};
        auto local_vector{rotation.inverse() * global_vector};

        auto local_angle_to_goal{atan2(local_vector.y(), local_vector.x())};

        auto distance_to_goal{sqrt(pow(current_goal.point.x - t_pose.position.x, 2) +
                                   pow(current_goal.point.y - t_pose.position.y, 2))};

        auto speed_linear{0.4};

        if (t_status == StatusModule::Status::FOLLOWING) {
//            auto n{-(0.32 * ((FOLLOW_THRESHOLD / 1000) - 0.2))};
            speed_linear = 1.5 * (distance_to_target / 1000) - 2.7;

            // maximum of 0.6 m/s linear velocity
            if (speed_linear > 0.6)
                speed_linear = 0.6;
        }

        // robot has reached the goal
        if (distance_to_goal < 0.3) {

            // input less acceleration
            speed.linear.x = m_current_linear - (m_current_linear / 100) * 40;
            speed.angular.z = m_current_angular - (m_current_angular / 100) * 40;

            m_waypoint_list->pop_front();

        } else if (local_angle_to_goal > 0.0)
            speed.angular.z = 1.0 * local_angle_to_goal;
        else if (local_angle_to_goal < -0.0)
            speed.angular.z = -1.0 * std::abs(local_angle_to_goal);

        speed.linear.x = speed_linear;
    }

        // waypoint list is empty
    else
        t_status = StatusModule::Status::SEARCHING;

    m_current_linear = speed.linear.x;
    m_current_angular = speed.angular.z;

    return speed;
}


geometry_msgs::Twist ControlModule::velocityCommand(const double t_angle, const geometry_msgs::Pose& t_pose,
                                                    const double t_x, const double t_y)
{
    auto speed{geometry_msgs::Twist{}};

    auto rotation{tf::Matrix3x3{
            cos(t_angle), -sin(t_angle), t_pose.position.x,
            sin(t_angle), cos(t_angle), t_pose.position.y,
            0.0, 0.0, 1.0
    }};

    auto global_vector{tf::Vector3{t_x, t_y, 1.0}};
    auto local_vector{rotation.inverse() * global_vector};

    auto local_angle_to_goal{atan2(local_vector.y(), local_vector.x())};

    if (local_angle_to_goal > 0.0)
        speed.angular.z = 1.0 * local_angle_to_goal;
    else if (local_angle_to_goal < 0.0)
        speed.angular.z = -1.0 * std::abs(local_angle_to_goal);

    return speed;
}
