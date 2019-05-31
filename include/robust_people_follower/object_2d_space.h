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


#ifndef ROBUST_PEOPLE_FOLLOWER_OBJECT_2D_SPACE_H
#define ROBUST_PEOPLE_FOLLOWER_OBJECT_2D_SPACE_H


#include <geometry_msgs/Pose.h>

#include "robust_people_follower/stamped_types.h"


class Object2DSpace
{
public:
    // implicit default constructor

    // pure virtual methods
    virtual void printInfo() const = 0;
    virtual void calculateAngle() = 0;
    virtual void calculateVelocity(double t_frequency) = 0;

    // setters
    inline geometry_msgs::Pose& pose() { return m_pose; }
    inline double& angle() { return m_angle; }

    // getters
    inline const geometry_msgs::Pose& pose() const { return m_pose; }
    inline const geometry_msgs::Pose& oldPose() const { return m_old_pose; }
    inline const double velocity() const { return m_velocity; }
    inline const double angle() const { return m_angle; }

    inline void updatePose() { m_old_pose = m_pose; }

protected:
    geometry_msgs::Pose m_pose{};
    geometry_msgs::Pose m_old_pose{};
    double m_velocity{};
    double m_angle{};
};


#endif //ROBUST_PEOPLE_FOLLOWER_OBJECT_2D_SPACE_H
