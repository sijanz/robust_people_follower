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


#include <tf/LinearMath/Matrix3x3.h>
#include <robust_people_follower/status_module.h>
#include "robust_people_follower/tracking_module.h"


TrackingModule::TrackingModule() : m_target{}
{
    m_tracked_persons = std::make_shared<std::vector<Person>>();
}


void TrackingModule::processSkeletonData(const body_tracker_msgs::Skeleton& t_skeleton,
                                         const geometry_msgs::Pose& t_robot_pose, StatusModule::Status& t_status)
{
    auto p{std::find(m_tracked_persons->begin(), m_tracked_persons->end(), t_skeleton.body_id)};

    // person is already in the list
    if (p != m_tracked_persons->end()) {

        // update information
        p->skeleton() = t_skeleton;

        auto q{tf::Quaternion{t_robot_pose.orientation.x, t_robot_pose.orientation.y, t_robot_pose.orientation.z,
                              t_robot_pose.orientation.w}};
        auto robot_angle{q.getAngle()};
        p->calculateAbsolutePosition(t_robot_pose.position.x, t_robot_pose.position.y, robot_angle);
        p->calculateAngle();
        p->calculateVelocity(10.0);
        p->updatePose();

        // target
        if (p->target()) {
            m_target = *p;

            // check for gestures
            if (t_skeleton.gesture == 2 && p->correctHandHeight()) {
                if (p->gestureBegin() == ros::Time{0})
                    p->gestureBegin() = ros::Time::now();

            } else {
                if (p->gestureBegin() != ros::Time{0}) {

                    /*
                    // TODO: figure out how to play a sound
                    // target chooses to stop being followed
                    if (ros::Time::now().sec - p->gestureBegin().sec >= 3) {
                        p->target() = false;
                        m_robot.target() = Person{body_tracker_msgs::Skeleton{}};
                        m_robot.status() = Robot::Status::WAITING;
                        m_robot.waypoints()->clear();
                        p->gestureBegin() = ros::Time{0};
                    }
                     */
                }
            }
        }

            // other persons
        else {

            // FIXME: doesn't work properly
            // check for gestures
            if (t_skeleton.gesture == 2 && p->correctHandHeight()) {
                if (p->gestureBegin() == ros::Time{0})
                    p->gestureBegin() = ros::Time::now();

            } else {
                if (p->gestureBegin() != ros::Time{0}) {

                    // TODO: figure out how to play a sound
                    // new target selected after 3 seconds of closing both hands
                    if (ros::Time::now().sec - p->gestureBegin().sec >= 3) {
                        p->target() = true;
                        m_target = *p;
                        t_status = StatusModule::Status::FOLLOWING;

                        // reset gesture beginning time
                        p->gestureBegin() = ros::Time{0};
                    }
                }
            }
        }
    }

        // add new person if skeleton id is not in list
    else
        m_tracked_persons->emplace_back(Person{t_skeleton});
}


void TrackingModule::managePersonList()
{
    auto it = m_tracked_persons->begin();
    while (it != m_tracked_persons->end()) {
        if (it->distance() == 0 && it->yDeviation() == 0)
            it = m_tracked_persons->erase(it);
        else
            ++it;
    }
}
