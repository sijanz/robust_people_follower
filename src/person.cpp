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

#include "robust_people_follower/person.h"


Person::Person(const body_tracker_msgs::Skeleton& t_skeleton)
        : m_is_target{}, m_skeleton{t_skeleton}, m_gesture_begin{}, m_mean_velocity{}, m_mean_angle{},
          m_velocities{new std::vector<VelocityStamped>{}}, m_mean_velocities{std::deque<VelocityStamped>{}},
          m_angles{new std::vector<AngleStamped>{}}, m_mean_angles{std::deque<AngleStamped>{}} {}


void Person::printInfo() const
{
    ROS_INFO("id: %d, is target: %d, distance: %f, gestures: %d, correct height: %d",
             m_skeleton.body_id, m_is_target, m_skeleton.centerOfMass.x, m_skeleton.gesture, correctHandHeight());
}


void Person::printVerboseInfo() const
{
    ROS_INFO_STREAM("id : " << m_skeleton.body_id);
    ROS_INFO_STREAM("  velocity: " << m_velocity);
    ROS_INFO_STREAM("  mean velocity: " << m_mean_velocity);
    ROS_INFO_STREAM("  mean_velocities size: " << m_mean_velocities.size());
    ROS_INFO_STREAM("  mean angle: " << m_mean_angle);
    ROS_INFO_STREAM("  theta: " << m_angle);
    ROS_INFO_STREAM("  distance: " << m_skeleton.centerOfMass.x);
    ROS_INFO_STREAM("  delta-y: " << m_skeleton.centerOfMass.y << "\n");
}


void Person::calculateAbsolutePosition(const double t_robot_x, const double t_robot_y, const double t_robot_angle)
{
    auto rotation{tf::Matrix3x3{
            cos(t_robot_angle), -sin(t_robot_angle), t_robot_x,
            sin(t_robot_angle), cos(t_robot_angle), t_robot_y,
            0.0, 0.0, 1.0
    }};

    auto local_vector{tf::Vector3{(m_skeleton.centerOfMass.x / 1000), (m_skeleton.centerOfMass.y / 1000), 1.0}};
    auto global_vector{rotation * local_vector};

    m_pose.position.x = global_vector.x();
    m_pose.position.y = global_vector.y();
}


bool Person::correctHandHeight() const
{
    return m_skeleton.joint_position_left_hand.z > m_skeleton.joint_position_spine_top.z;
}


void Person::calculateAngle()
{

    // FIXME: use abs?
    m_angle = std::atan2((m_pose.position.y - m_old_pose.position.y),
                         (m_pose.position.x - m_old_pose.position.x));

    auto it = m_angles->begin();
    while (it != m_angles->end()) {
        if (ros::Time::now() - it->stamp > ros::Duration{2})
            it = m_angles->erase(it);
        else
            ++it;
    }

    m_angles->emplace_back(AngleStamped{m_angle, ros::Time::now()});

    // TODO: test
    // TODO: only use last 5 seconds
    // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
    auto count{0};
    auto sum_sin{0.0};
    auto sum_cos{0.0};
    for (const auto& as : *m_angles) {
        if (ros::Time::now() - as.stamp <= ros::Duration{5}) {
            sum_sin += sin(as.angle);
            sum_cos += cos(as.angle);
            ++count;
        }
    }

    m_mean_angle = atan2(((1.0 / count) * sum_sin), ((1.0 / count) * sum_cos));
    m_mean_angles.emplace_back(AngleStamped{m_mean_angle, ros::Time::now()});

    if (m_mean_angles.size() > 200)
        m_mean_angles.pop_front();
}


void Person::calculateVelocity(const double t_frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / t_frequency);

    m_velocities->emplace_back(VelocityStamped{m_velocity, ros::Time::now()});

    auto count{0};
    auto sum{0.0};
    for (const auto& vs : *m_velocities) {
        if (ros::Time::now() - vs.stamp <= ros::Duration{5}) {
            sum += vs.velocity;
            ++count;
        }
    }

    m_mean_velocity = sum / count;

    // FIXME: not good, find better way to filter out high values
    if (m_mean_velocity < 7.0)
        m_mean_velocities.emplace_back(VelocityStamped{m_mean_velocity, ros::Time::now()});

    if (m_mean_velocities.size() > 200)
        m_mean_velocities.pop_front();
}
