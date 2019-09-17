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


#ifndef ROBUST_PEOPLE_FOLLOWER_PERSON_H
#define ROBUST_PEOPLE_FOLLOWER_PERSON_H


#include <cmath>

#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Point32.h>
#include <deque>

#include "robust_people_follower/object_2d_space.h"


/**
 * Stores data related to a tracked person.
 */
class Person : public Object2DSpace
{
public:

    // special methods
    Person() = default;
    explicit Person(const body_tracker_msgs::Skeleton& t_skeleton);
    inline Person& operator=(const Person& rhs) = default;
    inline bool operator==(const int t_id) { return t_id == m_skeleton.body_id; }

    // inherited methods
    void printInfo() const override;

    // setters
    inline bool& target() { return m_is_target; }
    inline ros::Time& gestureBegin() { return m_gesture_begin; }

    // getters
    inline const bool target() const { return m_is_target; }
    inline const double distance() const { return m_skeleton.centerOfMass.x; }
    inline const double yDeviation() const { return m_skeleton.centerOfMass.y; }
    inline const double meanVelocity() const { return m_mean_velocity; }
    inline const double meanAngle() const { return m_mean_angle; }

    inline const double lastSeen() const
    {
        if (!m_poses->empty())
            return (ros::Time::now() - m_poses->at(m_poses->size() - 1).header.stamp).toSec();
    }

    inline const std::deque<VelocityStamped>& meanVelocities() const { return m_mean_velocities; }
    inline const std::deque<AngleStamped>& meanAngles() const { return m_mean_angles; }

    void printVerboseInfo() const;
    bool correctHandHeight() const;
    void updateState(const body_tracker_msgs::Skeleton& t_skeleton, const geometry_msgs::PoseStamped& t_robot_pose);


private:
    bool m_is_target{};
    body_tracker_msgs::Skeleton m_skeleton{};
    ros::Time m_gesture_begin{};
    std::shared_ptr<std::vector<geometry_msgs::PoseStamped>> m_poses{};
    double m_mean_velocity{};
    double m_mean_mean_velocity{};
    double m_mean_angle{};
    double m_mean_mean_angle{};
    std::shared_ptr<std::vector<VelocityStamped>> m_velocities{};
    std::deque<VelocityStamped> m_mean_velocities{};
    std::deque<VelocityStamped> m_mean_mean_velocities{};
    std::deque<AngleStamped> m_mean_angles{};
    std::deque<AngleStamped> m_mean_mean_angles{};

    void calculateAbsolutePosition(const geometry_msgs::PoseStamped& t_robot_pose);
    void applyMovingAverageFilter(double t_interval_length_sec);
};


#endif //ROBUST_PEOPLE_FOLLOWER_PERSON_H
