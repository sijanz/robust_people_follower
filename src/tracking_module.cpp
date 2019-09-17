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
                                         const geometry_msgs::PoseStamped& t_robot_pose, StatusModule::Status& t_status)
{
    if (checkForValidData(t_skeleton)) {
        auto p{std::find(m_tracked_persons->begin(), m_tracked_persons->end(), t_skeleton.body_id)};

        // person is already in the list
        if (p != m_tracked_persons->end()) {

            // update the state of the tracked person
            p->updateState(t_skeleton, t_robot_pose);

            // target
            if (p->target()) {

                // update target information
                m_target = *p;

                // check for gestures
                if (t_skeleton.gesture == 2 && p->correctHandHeight()) {
                    if (p->gestureBegin() == ros::Time{0})
                        p->gestureBegin() = ros::Time::now();
                    /*
                    else {

                     // TODO: test, clear waypoints
                        // target chooses to not being followed anymore
                        if (ros::Time::now().sec - p->gestureBegin().sec >= 3) {
                            p->target() = false;
                            m_target = Person{body_tracker_msgs::Skeleton{}};
                            t_status = StatusModule::Status::WAITING;
                            p->gestureBegin() = ros::Time{0};
                        }
                    }
                     */
                }
            }

                // other persons
            else {

                // FIXME: doesn't work properly
                // check for gestures
                if (t_skeleton.gesture == 2 && p->correctHandHeight()) {
                    if (p->gestureBegin() == ros::Time{0})
                        p->gestureBegin() = ros::Time::now();
                    else {

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
}


bool TrackingModule::checkForValidData(const body_tracker_msgs::Skeleton& t_skeleton)
{
    return std::abs(t_skeleton.centerOfMass.y) < 1100;
}


// TODO: lose target if y_deviation is the same as t-1
void TrackingModule::checkForTargetLoss(StatusModule::Status& t_status)
{
    if (t_status == StatusModule::Status::FOLLOWING && target().lastSeen() > 0.05) {
        t_status = StatusModule::Status::LOS_LOST;

        // DEBUG
        ROS_INFO_STREAM("target lost!");
    }
}


void TrackingModule::managePersonList()
{
    auto it = m_tracked_persons->begin();
    while (it != m_tracked_persons->end()) {
        if (it->lastSeen() > 0.1 || it->lastSeen() == 0)
            it = m_tracked_persons->erase(it);
        else
            ++it;
    }
}
