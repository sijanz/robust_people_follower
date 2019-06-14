#include <cmath>

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


// TODO: update constructor (look at new members!)
Robot::Robot()
        : m_status{Status::WAITING}, m_waypoint_list{new std::deque<geometry_msgs::PointStamped>{}},
          m_last_waypoint_time{}, m_estimated_target_position{}, m_target{},
          m_tracked_persons{new std::vector<Person>} {}


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
    ROS_INFO("  theta: %f", m_angle);
    ROS_INFO("  predicted target position:");
    ROS_INFO("    x: %f", m_estimated_target_position.x);
    ROS_INFO("    y: %f\n", m_estimated_target_position.y);
}


void Robot::addNewWaypoint(const int t_times_per_second)
{
    if (m_waypoint_list->size() > 100)
        m_waypoint_list->pop_front();

    if (ros::Time::now() - m_last_waypoint_time > ros::Duration(0, (1000000000 / t_times_per_second))) {
        geometry_msgs::PointStamped position{};
        position.header.stamp = ros::Time::now();
        position.point.x = m_target.pose().position.x;
        position.point.y = m_target.pose().position.y;

        m_last_waypoint_time = ros::Time::now();
        m_waypoint_list->emplace_back(position);
    }
}


geometry_msgs::Twist Robot::velocityCommand(const double FOLLOW_THRESHOLD)
{
    auto distance_to_target{m_target.distance()};

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

        if (m_target.yDeviation() < -50) {
            speed.angular.z = -0.0025 * std::abs(m_target.yDeviation());
        } else if (m_target.yDeviation() > 50) {
            speed.angular.z = 0.0025 * m_target.yDeviation();
        } else
            speed.angular.z = 0;

        // target is under threshold, only keep him centered
    } else if (distance_to_target > 1000 && distance_to_target < FOLLOW_THRESHOLD) {

        if (m_target.yDeviation() < -50) {
            speed.angular.z = -0.0025 * std::abs(m_target.yDeviation());
        } else if (m_target.yDeviation() > 50) {
            speed.angular.z = 0.0025 * m_target.yDeviation();
        } else
            speed.angular.z = 0;

        // target is above threshold, follow him using waypoints
    } else if (!m_waypoint_list->empty()) {

        auto current_goal{m_waypoint_list->at(0)};

        tf::Matrix3x3 rotation{
                cos(m_angle), -sin(m_angle), m_pose.position.x,
                sin(m_angle), cos(m_angle), m_pose.position.y,
                0.0, 0.0, 1.0
        };

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

    m_angle = theta;
}


void Robot::calculateVelocity(const double t_frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / t_frequency);
}


// TODO: implement average filter
//
void Robot::estimateTargetPosition(const double x_l, const double y_l)
{

    // TODO: remove if new solution works
    /*
    auto distance{m_target.meanVelocity() * (ros::Time::now() - m_target.lastSeen()).toSec()};

    m_estimated_target_position.x = x_l + cos(m_target.meanAngle()) * distance;
    m_estimated_target_position.y = y_l + sin(m_target.meanAngle()) * distance;
     */

    // velocity
    auto velocity{0.0};
    for (const auto& vs : *m_target.meanVelocities())
        velocity += vs.velocity;

    velocity /= m_target.meanVelocities()->size();

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: velocity: " << velocity);


    // acceleration
    auto acceleration{0.0};
    auto first_matching_velocity{VelocityStamped{}};

    // DEBUG
    ROS_INFO_STREAM("mean velocities:");
    auto last_matching_velocity{m_target.meanVelocities()->at(m_target.meanVelocities()->size() - 1)};
    for (const auto& vs : *m_target.meanVelocities()) {
        if (m_target.lastSeen() - vs.stamp > ros::Duration{1}) {

            // DEBUG
            ROS_INFO_STREAM(vs.velocity);
            first_matching_velocity = vs;
            break;
        }
    }

    acceleration = (last_matching_velocity.velocity - first_matching_velocity.velocity)
                   / (last_matching_velocity.stamp - first_matching_velocity.stamp).toSec();

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: acceleration: " << acceleration);

    // FIXME: acceleration cannot be negative
//    if (acceleration < 0.0)
//        acceleration = 0.0;


    // yaw rate
    auto yaw_rate{0.0};
    auto first_matching_angle{AngleStamped{}};
    auto last_matching_angle{m_target.meanAngles()->at(m_target.meanAngles()->size() - 1)};
    for (const auto& as : *m_target.meanAngles()) {
        if (m_target.lastSeen() - as.stamp > ros::Duration{2}) {
            first_matching_angle = as;
            break;
        }
    }

    yaw_rate = (fmod(last_matching_angle.angle - first_matching_angle.angle + (3 * M_PI), 2 * M_PI) - M_PI)
               / (last_matching_angle.stamp - first_matching_angle.stamp).toSec();

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: yaw_rate: " << yaw_rate);

    auto theta{m_target.meanAngle()};

    //DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: theta: " << theta);

    auto delta_t{(ros::Time::now() - m_target.lastSeen()).toSec()};

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: delta_t: " << delta_t);

    auto x_t{(1 / pow(yaw_rate, 2.0)) * ((velocity * yaw_rate + acceleration * yaw_rate * delta_t)
                                         * sin(theta + yaw_rate * delta_t)
                                         + acceleration * cos(theta + yaw_rate * delta_t)
                                         - velocity * yaw_rate * sin(theta) - acceleration * cos(theta))};

    auto y_t{(1 / pow(yaw_rate, 2.0)) * ((-velocity * yaw_rate - acceleration * yaw_rate * delta_t)
                                         * cos(theta + yaw_rate * delta_t)
                                         + acceleration * sin(theta + yaw_rate * delta_t)
                                         + velocity * yaw_rate * cos(theta) - acceleration * sin(theta))};

    // TODO: frequency as parameter
    auto estimated_velocity{sqrt(pow((m_old_estimated_target_position.x - m_estimated_target_position.x), 2)
                                 + pow((m_old_estimated_target_position.y - m_estimated_target_position.y), 2))
                            / (1.0 / 10.0)};

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: estimated_velocity: " << estimated_velocity);

    if (estimated_velocity < 0.1 && (ros::Time::now() - m_target.lastSeen()) > ros::Duration{1})
        m_target_stop = true;

    if (!m_target_stop) {
        m_estimated_target_position.x = x_l + x_t;
        m_estimated_target_position.y = y_l + y_t;
    }

    m_old_estimated_target_position = m_estimated_target_position;
}


// TODO: add weight function
void Robot::reIdentify()
{
    auto min_distance{0.0};
    if (!m_tracked_persons->empty()) {
        min_distance = sqrt(
                pow((m_tracked_persons->at(0).pose().position.x - m_estimated_target_position.x), 2)
                + pow((m_tracked_persons->at(0).pose().position.y - m_estimated_target_position.y), 2));

        for (auto& p : *m_tracked_persons) {
            if (sqrt(pow((p.pose().position.x - m_estimated_target_position.x), 2)
                     + pow((p.pose().position.y - m_estimated_target_position.y), 2)) <
                min_distance) {
                min_distance = sqrt(pow((p.pose().position.x - m_estimated_target_position.x), 2)
                                    + pow((p.pose().position.y - m_estimated_target_position.y), 2));
                p.target() = true;
                m_target = p;
                m_status = Status::FOLLOWING;
            }
        }
    }
}


void Robot::managePersonList()
{
    auto it = m_tracked_persons->begin();
    while (it != m_tracked_persons->end()) {
        if (it->distance() == 0 && it->yDeviation() == 0)
            it = m_tracked_persons->erase(it);
        else
            ++it;
    }
}
