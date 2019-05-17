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


#ifndef ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H
#define ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H


#include <math.h>
#include <deque>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "person.h"


/**
 * Stores data related to the roboter.
 */
class Turtlebot
{
public:

    enum Status
    {
        WAITING = 0,
        FOLLOWING = 1,
        SEARCHING = 2
    };

    Turtlebot();
    void printTurtlebotInfo() const;
    Turtlebot::Status getStatus() const;
    double getAngle() const;
    geometry_msgs::Pose getPose();
    void setStatus(Turtlebot::Status t_status);
    void setPose(const geometry_msgs::Pose& t_pose);
    void updateOldPose();
    void calculateVelocity(double t_frequency);
    geometry_msgs::Twist& setVelocityCommand(const Person& t_target,
                                             std::deque<geometry_msgs::PointStamped>& t_goal_list,
                                             geometry_msgs::Twist& t_msg);

private:
    Turtlebot::Status m_status;
    double m_velocity;
    geometry_msgs::Pose m_pose;
    geometry_msgs::Pose m_old_pose;
    double m_angle;
    double m_current_linear;
    double m_current_angular;

    void calculateAngle();
};

#endif //ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H
