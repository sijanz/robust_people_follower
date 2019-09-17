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


#include <geometry_msgs/PointStamped.h>
#include <angles/angles.h>

#include "robust_people_follower/recovery_module.h"
#include "robust_people_follower/status_module.h"


void RecoveryModule::predictTargetPosition(const Person& t_target, const double t_interval_sec)
{
    // velocity
    auto velocity{t_target.meanVelocity()};

    // DEBUG
    ROS_INFO_STREAM("velocity: " << velocity);


    // acceleration
    auto delta_v{0.0};
    auto delta_t_v{0.0};
    for (int i = 0; i < t_target.meanVelocities().size() - 2; ++i) { // because last entry is way too high
        if (t_target.pose().header.stamp - t_target.meanVelocities()[i].stamp < ros::Duration{t_interval_sec}) {
            delta_v += t_target.meanVelocities()[i + 1].velocity - t_target.meanVelocities()[i].velocity;
            delta_t_v += t_target.meanVelocities()[i + 1].stamp.toSec() -
                         t_target.meanVelocities()[i].stamp.toSec();
        }
    }

    auto acceleration{delta_v / delta_t_v};

    // DEBUG
    ROS_INFO_STREAM("acceleration: " << acceleration);


    // yaw rate
    auto delta_a{0.0};
    auto delta_t_a{0.0};
    for (int i = 0; i < t_target.meanAngles().size() - 1; ++i) {
        if (t_target.pose().header.stamp - t_target.meanAngles()[i].stamp < ros::Duration{t_interval_sec}) {
            delta_a += angles::shortest_angular_distance(t_target.meanAngles()[i].angle,
                                                         t_target.meanAngles()[i + 1].angle);
            delta_t_a += t_target.meanAngles()[i + 1].stamp.toSec() - t_target.meanAngles()[i].stamp.toSec();
        }
    }

    auto yaw_rate{delta_a / delta_t_a};

    // DEBUG
    ROS_INFO_STREAM("yaw rate: " << yaw_rate);


    // theta
    auto theta{t_target.meanAngle()};

    //DEBUG
    ROS_INFO_STREAM("theta: " << theta);


    // delta_t
    auto delta_t{t_target.lastSeen()};

    // DEBUG
    ROS_INFO_STREAM("delta t: " << delta_t);


    // estimated position
    auto x_t{(1 / pow(yaw_rate, 2.0)) * ((velocity * yaw_rate + acceleration * yaw_rate * delta_t)
                                         * sin(theta + yaw_rate * delta_t)
                                         + acceleration * cos(theta + yaw_rate * delta_t)
                                         - velocity * yaw_rate * sin(theta) - acceleration * cos(theta))};

    auto y_t{(1 / pow(yaw_rate, 2.0)) * ((-velocity * yaw_rate - acceleration * yaw_rate * delta_t)
                                         * cos(theta + yaw_rate * delta_t)
                                         + acceleration * sin(theta + yaw_rate * delta_t)
                                         + velocity * yaw_rate * cos(theta) - acceleration * sin(theta))};

    if (!m_prediction_stop) {
        m_predicted_target_position.x = t_target.pose().pose.position.x + x_t;
        m_predicted_target_position.y = t_target.pose().pose.position.y + y_t;


        // estimated velocity
        if (!(m_old_predicted_target_position.x == 0 && m_old_predicted_target_position.y == 0))
            m_predicted_velocity = sqrt(pow((m_old_predicted_target_position.x - m_predicted_target_position.x), 2)
                                        + pow((m_old_predicted_target_position.y - m_predicted_target_position.y), 2))
                                   / (1 / 10.0); // TODO: do not use frequency

        // DEBUG
        ROS_INFO_STREAM("predicted old position: [" << m_old_predicted_target_position.x << ", " <<
                                                    m_old_predicted_target_position.y << "]");
        ROS_INFO_STREAM("predicted position: [" << m_predicted_target_position.x << ", " <<
                                                m_predicted_target_position.y << "]");
        ROS_INFO_STREAM("predicted velocity: " << m_predicted_velocity);

        m_old_predicted_target_position = m_predicted_target_position;

        // stop estimation before velocity goes "negative"
        if (m_predicted_velocity < 0.1 && t_target.lastSeen() > 0.2)
            m_prediction_stop = true;
    }

    m_prediction_radius = velocity * delta_t;
}


void RecoveryModule::reIdentify(Person& t_target, const std::shared_ptr<std::vector<Person>>& t_tracked_persons,
                                const std::shared_ptr<std::deque<geometry_msgs::PointStamped>>& t_waypoint_list,
                                StatusModule::Status& t_status)
{
    auto min_distance{std::numeric_limits<double>::max()};

    // named lambda function to calculate the Euclidean distance between two points
    auto distance = [](const geometry_msgs::Point& p1, const geometry_msgs::Point32& p2) {
        return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
    };

    for (auto& p : *t_tracked_persons) {
        if (distance(p.pose().pose.position, m_predicted_target_position) < min_distance) {
            min_distance = distance(p.pose().pose.position, m_predicted_target_position);

            // person is in radius and nearest to the predicted position, set target
            if (distance(p.pose().pose.position, m_predicted_target_position) < m_prediction_radius) {
                t_waypoint_list->clear();
                m_old_predicted_target_position = geometry_msgs::Point32{};
                m_predicted_target_position = geometry_msgs::Point32{};
                m_prediction_radius = 0.0;
                p.target() = true;
                t_target = p;
                t_status = StatusModule::Status::FOLLOWING;
            }
        }
    }
}
