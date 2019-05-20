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


Person::Person() : m_is_target(false), m_gesture_begin(ros::Time(0)), m_skeleton(body_tracker_msgs::Skeleton{}) {}


Person::Person(const body_tracker_msgs::Skeleton& t_skeleton) : m_is_target(false), m_gesture_begin(ros::Time(0)),
                                                                m_skeleton(t_skeleton) {}


Person& Person::operator=(const Person& rhs)
{
    m_is_target = rhs.m_is_target;
    m_velocity = rhs.m_velocity;
    m_angle_radian = rhs.m_angle_radian;
    m_gesture_begin = rhs.m_gesture_begin;
    m_pose = rhs.m_pose;
    m_old_pose = rhs.m_old_pose;
    m_skeleton = rhs.m_skeleton;
}


void Person::printInfo() const
{
    ROS_INFO("id: %d, is target: %d, distance: %f, gestures: %d, height: %d",
             m_skeleton.body_id, m_is_target, m_skeleton.centerOfMass.x, m_skeleton.gesture, hasCorrectHandHeight());
}


void Person::printVerboseInfo() const
{
    ROS_INFO("id: %d", m_skeleton.body_id);
    ROS_INFO("  velocity: %f", m_velocity);
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


void Person::calculateAbsolutePosition(double t_robot_x, double t_robot_y, double t_robot_angle)
{
    m_pose.position.x = t_robot_x + (cos(t_robot_angle) * (m_skeleton.centerOfMass.x / 1000) -
                                     sin(t_robot_angle) * (m_skeleton.centerOfMass.y / 1000));
    m_pose.position.y = t_robot_y + (sin(t_robot_angle) * (m_skeleton.centerOfMass.x / 1000) +
                                     cos(t_robot_angle) * (m_skeleton.centerOfMass.y / 1000));
}


bool Person::hasCorrectHandHeight() const
{
    return m_skeleton.joint_position_left_hand.z > m_skeleton.joint_position_spine_top.z;
}


void Person::calculateAngle()
{
    m_angle_radian = std::atan2((m_pose.position.y - m_old_pose.position.y),
                                (m_pose.position.x - m_old_pose.position.x));
}
