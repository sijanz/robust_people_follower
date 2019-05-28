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


#include <tf/transform_datatypes.h>

#include "robust_people_follower/robot.h"


Robot::Robot() :
        m_status{Status::WAITING}, m_waypoint_list{new std::deque<geometry_msgs::PointStamped>{}},
        m_last_waypoint_time{}, m_current_linear{}, m_current_angular{}, m_estimated_target_position{} {}


void Robot::printInfo() const
{
    auto status_string{""};
    switch (m_status) {
        case 0:
            status_string = "WAITING";
            break;
        case 1:
            status_string = "FOLLOWING";
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
    ROS_INFO("  theta: %f", m_angle_radian);
    ROS_INFO("  predicted target position:");
    ROS_INFO("    x: %f", m_estimated_target_position.x);
    ROS_INFO("    y: %f\n", m_estimated_target_position.y);
}


void Robot::addNewWaypoint(const Person& t_target, int t_times_per_second)
{
    if (m_waypoint_list->size() > 100)
        m_waypoint_list->pop_front();

    if (ros::Time::now() - m_last_waypoint_time > ros::Duration(0, (1000000000 / t_times_per_second))) {
        geometry_msgs::PointStamped position{};
        position.header.stamp = ros::Time::now();
        position.point.x = t_target.pose().position.x;
        position.point.y = t_target.pose().position.y;

        m_last_waypoint_time = ros::Time::now();
        m_waypoint_list->emplace_back(position);
    }
}


geometry_msgs::Twist Robot::velocityCommand(const Person& t_target, const double FOLLOW_THRESHOLD)
{
    auto distance_to_target{t_target.distance()};

    // FIXME: ugly, only temporary fix
    if (distance_to_target == 0)
        distance_to_target = FOLLOW_THRESHOLD + 200;

    if (distance_to_target < FOLLOW_THRESHOLD)
        m_waypoint_list->clear();

    if (m_status == Status::LOS_LOST && m_waypoint_list->empty())
        m_status = Status::SEARCHING;

    geometry_msgs::Twist speed{};

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

        tf::Matrix3x3 rotation{};
        rotation.setValue(cos(m_angle_radian), -sin(m_angle_radian), m_pose.position.x,
                          sin(m_angle_radian), cos(m_angle_radian), m_pose.position.y,
                          0.0, 0.0, 1.0);
        tf::Vector3 global_vector{current_goal.point.x, current_goal.point.y, 1.0};

        auto local_vector{rotation.inverse() * global_vector};
        auto local_angle_to_goal{atan2(local_vector.y(), local_vector.x())};

        auto distance_to_goal{sqrt(pow(current_goal.point.x - m_pose.position.x, 2) +
                                   pow(current_goal.point.y - m_pose.position.y, 2))};

        auto speed_linear{0.3};

        if (m_status == Status::FOLLOWING) {
            auto n{-(0.32 * ((FOLLOW_THRESHOLD / 1000) - 0.2))};
            speed_linear = 0.32 * (distance_to_target / 1000) + n;

            // maximum of 0.6 m/s linear velocity
            if (speed_linear > 0.6)
                speed_linear = 0.6;
        }

        // roboter has reached the goal
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
        m_status = Status::SEARCHING;

    m_current_linear = speed.linear.x;
    m_current_angular = speed.angular.z;

    return speed;
}


void Robot::calculateAngle()
{
    tf::Quaternion q{m_pose.orientation.x, m_pose.orientation.y, m_pose.orientation.z, m_pose.orientation.w};
    tf::Matrix3x3 m{q};

    auto roll{0.0}, pitch{0.0}, theta{0.0};
    m.getRPY(roll, pitch, theta);

    m_angle_radian = theta;
}


void Robot::calculateVelocity(double t_frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / t_frequency);
}


void Robot::estimateTargetPosition(const Person& t_target, const double t_x, const double t_y)
{
    auto distance{t_target.averageVelocity() * (ros::Time::now() - t_target.lastSeen()).toSec()};

    m_estimated_target_position.x = t_x + cos(t_target.meanAngle()) * distance;
    m_estimated_target_position.y = t_y + sin(t_target.meanAngle()) * distance;
}
