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

#include "robust_people_follower/object_2d_space.h"


/**
 * Stores data related to a tracked person.
 */
class Person : public Object2DSpace
{
public:
    Person();
    explicit Person(const body_tracker_msgs::Skeleton& t_skeleton);
    Person& operator=(const Person& rhs);
    void printInfo() const override;
    void printVerboseInfo() const;
    int getId() const;
    bool isTarget() const;
    body_tracker_msgs::Skeleton getSkeleton() const;
    int getGestureBegin() const;
    double getDistance() const;
    double getYDeviation() const;
    bool hasCorrectHandHeight() const;
    void setTarget(bool t_is_target);
    void setSkeleton(const body_tracker_msgs::Skeleton& t_skeleton);
    void setGestureBegin(const ros::Time& t_gesture_begin);
    void calculateAbsolutePosition(double t_robot_x, double t_robot_y, double t_robot_angle);
    void calculateAngle() override;

private:
    bool m_is_target;
    body_tracker_msgs::Skeleton m_skeleton;
    ros::Time m_gesture_begin;
};


#endif //ROBUST_PEOPLE_FOLLOWER_PERSON_H
