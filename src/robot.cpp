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


#include <cmath>
#include <tf/transform_datatypes.h>

#include "robust_people_follower/robot.h"


Robot::Robot()
        : m_status{Status::WAITING}, m_waypoint_list{new std::deque<geometry_msgs::PointStamped>{}},
          m_last_waypoint_time{}, m_old_estimated_target_position{}, m_estimated_target_position{},
          m_target{body_tracker_msgs::Skeleton{}}, m_estimation_stop{false},
          m_tracked_persons{new std::vector<Person>}, m_current_linear{}, m_current_angular{} {}


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
        auto position{geometry_msgs::PointStamped{}};
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

    auto speed{geometry_msgs::Twist{}};

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

        auto rotation{tf::Matrix3x3{
                cos(m_angle), -sin(m_angle), m_pose.position.x,
                sin(m_angle), cos(m_angle), m_pose.position.y,
                0.0, 0.0, 1.0
        }};

        auto global_vector{tf::Vector3{current_goal.point.x, current_goal.point.y, 1.0}};
        auto local_vector{rotation.inverse() * global_vector};

        auto local_angle_to_goal{atan2(local_vector.y(), local_vector.x())};

        auto distance_to_goal{sqrt(pow(current_goal.point.x - m_pose.position.x, 2) +
                                   pow(current_goal.point.y - m_pose.position.y, 2))};

        auto speed_linear{0.4};

        if (m_status == Status::FOLLOWING) {
//            auto n{-(0.32 * ((FOLLOW_THRESHOLD / 1000) - 0.2))};
            speed_linear = 1.5 * (distance_to_target / 1000) - 2.7;

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


geometry_msgs::Twist Robot::velocityCommand(const double t_x, const double t_y)
{
    auto speed{geometry_msgs::Twist{}};

    auto rotation{tf::Matrix3x3{
            cos(m_angle), -sin(m_angle), m_pose.position.x,
            sin(m_angle), cos(m_angle), m_pose.position.y,
            0.0, 0.0, 1.0
    }};

    auto global_vector{tf::Vector3{t_x, t_y, 1.0}};
    auto local_vector{rotation.inverse() * global_vector};

    auto local_angle_to_goal{atan2(local_vector.y(), local_vector.x())};


    // DEBUG
    ROS_INFO_STREAM("velocityCommand: local_angle_to_goal: " << local_angle_to_goal);


    if (local_angle_to_goal > 0.0)
        speed.angular.z = 1.0 * local_angle_to_goal;
    else if (local_angle_to_goal < 0.0)
        speed.angular.z = -1.0 * std::abs(local_angle_to_goal);

    return speed;
}


void Robot::calculateAngle()
{
    auto q{tf::Quaternion{m_pose.orientation.x, m_pose.orientation.y, m_pose.orientation.z, m_pose.orientation.w}};
    auto m{tf::Matrix3x3{q}};

    auto roll{0.0}, pitch{0.0}, theta{0.0};
    m.getRPY(roll, pitch, theta);

    m_angle = theta;
}


void Robot::calculateVelocity(const double t_frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / t_frequency);
}


void Robot::estimateTargetPosition(const double t_last_x, const double t_last_y, const double t_frequency)
{
    // velocity
    auto velocity{0.0};
    int count{0};
    for (const auto& vs : m_target.meanVelocities()) {
        if (m_target.lastSeen() - vs.stamp < ros::Duration{5}) {
            velocity += vs.velocity;
            ++count;
        }
    }

    velocity /= count;

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: velocity: " << velocity);


    // acceleration
    auto delta_v{0.0};
    auto delta_t_v{0.0};
    for (int i = 0; i < m_target.meanVelocities().size() - 2; ++i) { // because last entry is way too high
        if (m_target.lastSeen() - m_target.meanVelocities()[i].stamp < ros::Duration{0.5}) {
            delta_v += m_target.meanVelocities()[i + 1].velocity - m_target.meanVelocities()[i].velocity;
            delta_t_v += m_target.meanVelocities()[i + 1].stamp.toSec() - m_target.meanVelocities()[i].stamp.toSec();
        }
    }

    auto acceleration{delta_v / delta_t_v};

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: acceleration: " << acceleration);


    // yaw rate
    // https://gamedev.stackexchange.com/questions/4467/comparing-angles-and-working-out-the-difference/169509#169509
    auto delta_a{0.0};
    auto delta_t_a{0.0};
    for (int i = 0; i < m_target.meanAngles().size() - 1; ++i) {
        if (m_target.lastSeen() - m_target.meanAngles()[i].stamp < ros::Duration{0.5}) {
            delta_a += fmod(m_target.meanAngles()[i + 1].angle - m_target.meanAngles()[i].angle
                            + (3 * M_PI), 2 * M_PI) - M_PI;
            delta_t_a += m_target.meanAngles()[i + 1].stamp.toSec() - m_target.meanAngles()[i].stamp.toSec();
        }
    }

    auto yaw_rate{delta_a / delta_t_a};

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: yaw_rate: " << yaw_rate);


    // theta
    auto theta{m_target.meanAngle()};

    //DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: theta: " << theta);


    // delta_t
    auto delta_t{(ros::Time::now() - m_target.lastSeen()).toSec()};

    // DEBUG
    ROS_INFO_STREAM("estimateTargetPosition: delta_t: " << delta_t);


    // estimated position
    auto x_t{(1 / pow(yaw_rate, 2.0)) * ((velocity * yaw_rate + acceleration * yaw_rate * delta_t)
                                         * sin(theta + yaw_rate * delta_t)
                                         + acceleration * cos(theta + yaw_rate * delta_t)
                                         - velocity * yaw_rate * sin(theta) - acceleration * cos(theta))};

    auto y_t{(1 / pow(yaw_rate, 2.0)) * ((-velocity * yaw_rate - acceleration * yaw_rate * delta_t)
                                         * cos(theta + yaw_rate * delta_t)
                                         + acceleration * sin(theta + yaw_rate * delta_t)
                                         + velocity * yaw_rate * cos(theta) - acceleration * sin(theta))};

    if (!m_estimation_stop) {
        m_estimated_target_position.x = t_last_x + x_t;
        m_estimated_target_position.y = t_last_y + y_t;


        // estimated velocity
        auto estimated_velocity{sqrt(pow((m_old_estimated_target_position.x - m_estimated_target_position.x), 2)
                                     + pow((m_old_estimated_target_position.y - m_estimated_target_position.y), 2))
                                / (1 / t_frequency)};

        // DEBUG
        ROS_INFO_STREAM(
                "estimateTargetPosition: estimated old position: [" << m_old_estimated_target_position.x << ", " <<
                                                                    m_old_estimated_target_position.y << "]");
        ROS_INFO_STREAM("estimateTargetPosition: estimated position: [" << m_estimated_target_position.x << ", " <<
                                                                        m_estimated_target_position.y << "]");
        ROS_INFO_STREAM("estimateTargetPosition: estimated_velocity: " << estimated_velocity);

        m_old_estimated_target_position = m_estimated_target_position;

        // stop estimation before velocity goes "negative"
        if (estimated_velocity < 0.1 && (ros::Time::now() - m_target.lastSeen()) > ros::Duration{0.2})
            m_estimation_stop = true;
    }


    // TODO: better mathematical way? (-> Kalman filter)
    // estimation radius
    m_estimation_radius = 0.75 * velocity * delta_t;

    // DEBUG
    ROS_INFO_STREAM("estimation radius: " << m_estimation_radius);
}


void Robot::reIdentify()
{
    auto min_distance{std::numeric_limits<double>::max()};

    // named lambda function to calculate the Euclidean distance between two points
    auto distance = [](const geometry_msgs::Point& p1, const geometry_msgs::Point32& p2) {
        return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
    };

    for (auto& p : *m_tracked_persons) {
        if (distance(p.pose().position, m_estimated_target_position) < min_distance) {
            min_distance = distance(p.pose().position, m_estimated_target_position);

            // person is in radius and nearest to the predicted position, set target
            if (distance(p.pose().position, m_estimated_target_position) < m_estimation_radius) {
                m_waypoint_list->clear();
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
