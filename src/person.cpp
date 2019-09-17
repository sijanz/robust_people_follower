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
        : m_is_target{}, m_skeleton{t_skeleton}, m_gesture_begin{}, m_mean_velocity{},
          m_mean_angle{}, m_mean_velocities{std::deque<VelocityStamped>{}}, m_mean_angles{std::deque<AngleStamped>{}}
{
    m_poses = std::make_shared<std::vector<geometry_msgs::PoseStamped>>();
    m_velocities = std::make_shared<std::vector<VelocityStamped>>();
}


void Person::printInfo() const
{
    ROS_INFO("id: %d, is target: %d, distance: %f, velocity: %f, gestures: %d, correct height: %d, last seen: %f",
             m_skeleton.body_id, m_is_target, m_skeleton.centerOfMass.x, m_velocity, m_skeleton.gesture,
             this->correctHandHeight(), this->lastSeen());
}


void Person::printVerboseInfo() const
{
    ROS_INFO_STREAM("id : " << m_skeleton.body_id);
    if (!m_poses->empty()) {
        ROS_INFO_STREAM("  pose vector size: " << m_poses->size());
        ROS_INFO_STREAM("  last seen: " << this->lastSeen());
    }
    ROS_INFO_STREAM("  mean velocity: " << m_mean_velocity);
    ROS_INFO_STREAM("  mean mean veloctiy: " << m_mean_mean_velocity);
    ROS_INFO_STREAM("  mean angle: " << m_mean_angle * (180 / M_PI));
    ROS_INFO_STREAM("  mean mean angle: " << m_mean_mean_angle * (180 / M_PI));
    ROS_INFO_STREAM("  distance: " << m_skeleton.centerOfMass.x);
    ROS_INFO_STREAM("  delta-y: " << m_skeleton.centerOfMass.y << "\n");
}


void Person::updateState(const body_tracker_msgs::Skeleton& t_skeleton, const geometry_msgs::PoseStamped& t_robot_pose)
{
    if (!m_values_set) {

        m_skeleton = t_skeleton;
        calculateAbsolutePosition(t_robot_pose);

        // calculate angle
        auto angle{std::atan2((m_current_pose.pose.position.y - m_old_pose.pose.position.y),
                              (m_current_pose.pose.position.x - m_old_pose.pose.position.x))};
        auto q{tf::Quaternion{tf::createQuaternionFromYaw(angle)}};
        m_current_pose.pose.orientation.x = q.x();
        m_current_pose.pose.orientation.y = q.y();
        m_current_pose.pose.orientation.z = q.z();
        m_current_pose.pose.orientation.w = q.w();

        m_poses->emplace_back(m_current_pose);

        calculateVelocity();
        applyMovingAverageFilter(0.5);
        updatePose();
        m_values_set = true;
    }
}


void Person::calculateAbsolutePosition(const geometry_msgs::PoseStamped& t_robot_pose)
{
    if (m_skeleton.centerOfMass.x > 0) {
        auto robot_angle{yawFromPose(t_robot_pose)};

        auto rotation{tf::Matrix3x3{
                cos(robot_angle), -sin(robot_angle), t_robot_pose.pose.position.x,
                sin(robot_angle), cos(robot_angle), t_robot_pose.pose.position.y,
                0.0, 0.0, 1.0
        }};

        auto local_vector{tf::Vector3{(m_skeleton.centerOfMass.x / 1000), (m_skeleton.centerOfMass.y / 1000), 1.0}};
        auto global_vector{rotation * local_vector};

        m_current_pose.header.stamp = ros::Time::now();
        m_current_pose.pose.position.x = global_vector.x();
        m_current_pose.pose.position.y = global_vector.y();
    }
}


void Person::applyMovingAverageFilter(const double t_interval_length_sec)
{
    m_velocities->emplace_back(VelocityStamped{m_velocity, ros::Time::now()});

    // angle

    // calculate a new mean angle and place it in the vector
    // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
    auto count{0};
    auto sum_sin{0.0};
    auto sum_cos{0.0};
    for (const auto& ps : *m_poses) {
        if ((ros::Time::now() - ps.header.stamp) <= ros::Duration{t_interval_length_sec}) {

            auto current_pose{geometry_msgs::PoseStamped{}};
            current_pose.pose.orientation.x = ps.pose.orientation.x;
            current_pose.pose.orientation.y = ps.pose.orientation.y;
            current_pose.pose.orientation.z = ps.pose.orientation.z;
            current_pose.pose.orientation.w = ps.pose.orientation.w;

            sum_sin += sin(yawFromPose(current_pose));
            sum_cos += cos(yawFromPose(current_pose));

            ++count;
        }
    }

    m_mean_angle = atan2(((1.0 / count) * sum_sin), ((1.0 / count) * sum_cos));
    m_mean_angles.emplace_back(AngleStamped{m_mean_angle, ros::Time::now()});

    if (m_mean_angles.size() > 200)
        m_mean_angles.pop_front();

    count = 0;
    sum_sin = 0.0;
    sum_cos = 0.0;

    for (const auto& as : m_mean_angles) {
        if ((ros::Time::now() - as.stamp) <= ros::Duration{t_interval_length_sec}) {
            sum_sin += sin(as.angle);
            sum_cos += cos(as.angle);
            ++count;
        }
    }

    m_mean_mean_angle = atan2(((1.0 / count) * sum_sin), ((1.0 / count) * sum_cos));
    m_mean_mean_angles.emplace_back(AngleStamped{m_mean_mean_angle, ros::Time::now()});

    if (m_mean_mean_angles.size() > 200)
        m_mean_mean_angles.pop_front();


    // velocity
    // calculate and add new mean velocity
    count = 0;
    auto sum{0.0};
    for (const auto& vs : *m_velocities) {
        if ((ros::Time::now() - vs.stamp) <= ros::Duration{t_interval_length_sec}) {
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

    count = 0;
    sum = 0.0;
    for (const auto& vs : m_mean_velocities) {
        if ((ros::Time::now() - vs.stamp) <= ros::Duration{t_interval_length_sec}) {
            sum += vs.velocity;
            ++count;
        }
    }

    m_mean_mean_velocity = sum / count;
    m_mean_mean_velocities.emplace_back(VelocityStamped{m_mean_mean_velocity, ros::Time::now()});

    if (m_mean_mean_velocities.size() > 200)
        m_mean_mean_velocities.pop_front();
}


bool Person::correctHandHeight() const
{
    return m_skeleton.joint_position_left_hand.z > m_skeleton.joint_position_spine_top.z;
}
