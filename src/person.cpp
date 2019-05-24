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


#include "robust_people_follower/person.h"


Person::Person(const body_tracker_msgs::Skeleton& t_skeleton) :
        m_is_target{}, m_gesture_begin{}, m_skeleton{t_skeleton}, m_average_velocity{},
        m_velocities{new std::vector<VelocityStamped>{}}, m_angles{new std::vector<QuaternionStamped>{}} {}


void Person::printInfo() const
{
    ROS_INFO("id: %d, is target: %d, distance: %f, gestures: %d, height: %d",
             m_skeleton.body_id, m_is_target, m_skeleton.centerOfMass.x, m_skeleton.gesture, correctHandHeight());
}


void Person::printVerboseInfo() const
{
    ROS_INFO("id: %d", m_skeleton.body_id);
    ROS_INFO("  velocity: %f", m_velocity);
    ROS_INFO("  average velocity: %f", m_average_velocity);
    ROS_INFO("  average angle: %f", m_average_angle);
    ROS_INFO("  position (relative)");
    ROS_INFO("    x: %f", m_skeleton.joint_position_spine_top.x);
    ROS_INFO("    y: %f", m_skeleton.joint_position_spine_top.y);
    ROS_INFO("  position (absolute):");
    ROS_INFO("    x: %f", m_pose.position.x);
    ROS_INFO("    y: %f", m_pose.position.y);
    ROS_INFO("  theta: %f", m_angle_radian);
    ROS_INFO("  distance: %f", m_skeleton.centerOfMass.x);
    ROS_INFO("  y-deviation of center of mass: %f\n", m_skeleton.centerOfMass.y);
}


void Person::calculateAbsolutePosition(const double t_robot_x, const double t_robot_y, const double t_robot_angle)
{
    m_pose.position.x = t_robot_x + (cos(t_robot_angle) * (m_skeleton.centerOfMass.x / 1000) -
                                     sin(t_robot_angle) * (m_skeleton.centerOfMass.y / 1000));
    m_pose.position.y = t_robot_y + (sin(t_robot_angle) * (m_skeleton.centerOfMass.x / 1000) +
                                     cos(t_robot_angle) * (m_skeleton.centerOfMass.y / 1000));
}


bool Person::correctHandHeight() const
{
    return m_skeleton.joint_position_left_hand.z > m_skeleton.joint_position_spine_top.z;
}


void Person::calculateAngle()
{
    m_angle_radian = std::atan2((m_pose.position.y - m_old_pose.position.y),
                                (m_pose.position.x - m_old_pose.position.x));

    auto it = m_angles->begin();
    while (it != m_angles->end()) {
        if (ros::Time::now() - it->stamp > ros::Duration(2))
            it = m_angles->erase(it);
        else
            ++it;
    }

    m_angles->emplace_back(QuaternionStamped{m_angle_radian, ros::Time::now()});

    QuaternionStamped sum{};
    std::for_each(m_angles->begin(), m_angles->end(), [&sum](const QuaternionStamped& qs) { sum += qs; });
    sum /= m_angles->size();

    m_average_angle = sum.quaternion.getAngle();
}


void Person::calculateVelocity(double t_frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / t_frequency);

    auto it = m_velocities->begin();
    while (it != m_velocities->end()) {
        if (ros::Time::now() - it->stamp > ros::Duration(2))
            it = m_velocities->erase(it);
        else
            ++it;
    }

    m_velocities->emplace_back(VelocityStamped{m_velocity, ros::Time::now()});

    VelocityStamped sum{};
    std::for_each(m_velocities->begin(), m_velocities->end(), [&sum](const VelocityStamped& vs) { sum += vs; });
    sum /= m_velocities->size();

    m_average_velocity = sum.velocity;
}
