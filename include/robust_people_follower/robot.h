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


#ifndef ROBUST_PEOPLE_FOLLOWER_ROBOT_H
#define ROBUST_PEOPLE_FOLLOWER_ROBOT_H


#include <deque>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include "robust_people_follower/object_2d_space.h"
#include "robust_people_follower/person.h"


/**
 * Stores data related to the robot.
 */
class Robot : public Object2DSpace
{
public:

    // enumeration to represent the status of the robot
    enum Status : unsigned char
    {
        WAITING = 0,
        FOLLOWING = 1,
        LOS_LOST = 2,
        SEARCHING = 3
    };

    // special methods
    Robot();

    // inherited methods
    void calculateAngle() override;
    void calculateVelocity(double t_frequency) override;
    void printInfo() const override;

    // setters
    inline Robot::Status& status() { return m_status; }
    inline Person& target() { return m_target; }
    inline std::shared_ptr<std::vector<Person>> trackedPersons() { return m_tracked_persons; }

    // getters
    inline const Robot::Status status() const { return m_status; }
    inline const std::shared_ptr<std::deque<geometry_msgs::PointStamped>> waypoints() const { return m_waypoint_list; }
    inline const geometry_msgs::Point32 estimatedTargetPosition() const { return m_estimated_target_position; }
    inline const Person& target() const { return m_target; }
    inline const std::shared_ptr<std::vector<Person>> trackedPersons() const { return m_tracked_persons; }

    void addNewWaypoint(int t_times_per_second);
    void estimateTargetPosition(double t_x, double t_y);
    geometry_msgs::Twist velocityCommand(double FOLLOW_THRESHOLD);
    void reIdentify();
    void managePersonList();

private:
    Status m_status{Status::WAITING};
    std::shared_ptr<std::deque<geometry_msgs::PointStamped>> m_waypoint_list{};
    ros::Time m_last_waypoint_time{};
    geometry_msgs::Point32 m_estimated_target_position{};

    // instance of the target to follow
    Person m_target{};

    // list of persons in the frame
    std::shared_ptr<std::vector<Person>> m_tracked_persons{};
};


#endif //ROBUST_PEOPLE_FOLLOWER_ROBOT_H
