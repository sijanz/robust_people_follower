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


class Object2DSpace
{
public:
    Object2DSpace();
    virtual void printInfo() const = 0;
    geometry_msgs::Pose getPose() const;
    geometry_msgs::Pose getOldPose() const;
    double getVelocity() const;
    double getAngle() const;
    void setPose(const geometry_msgs::Pose& t_pose);
    void setOldPose(const geometry_msgs::Pose& t_old_pose);
    void setVelocity(double t_velocity);
    void setAngle(double t_angle);
    void updateOldPose();
    void calculateVelocity(double t_frequency);
    virtual void calculateAngle() = 0;

protected:
    geometry_msgs::Pose m_pose;
    geometry_msgs::Pose m_old_pose;
    double m_velocity;
    double m_angle;
};


#endif //ROBUST_PEOPLE_FOLLOWER_OBJECT_2D_SPACE_H
