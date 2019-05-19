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

#include "robust_people_follower/object_2d_space.h"


Object2DSpace::Object2DSpace() : m_pose(geometry_msgs::Pose{}), m_old_pose(geometry_msgs::Pose{}),
                                 m_velocity(0.0), m_angle(0.0)
{}


geometry_msgs::Pose Object2DSpace::getPose() const
{
    return m_pose;
}


geometry_msgs::Pose Object2DSpace::getOldPose() const
{
    return m_old_pose;
}


double Object2DSpace::getVelocity() const
{
    return m_velocity;
}


double Object2DSpace::getAngle() const
{
    return m_angle;
}


void Object2DSpace::setPose(const geometry_msgs::Pose& t_pose)
{
    m_pose = t_pose;
}


void Object2DSpace::setOldPose(const geometry_msgs::Pose& t_old_pose)
{
    m_old_pose = t_old_pose;
}


void Object2DSpace::setVelocity(double t_velocity)
{
    m_velocity = t_velocity;
}


void Object2DSpace::setAngle(double t_angle)
{
    m_angle = t_angle;
}


void Object2DSpace::updateOldPose()
{
    m_old_pose = m_pose;
}


void Object2DSpace::calculateVelocity(double t_frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / t_frequency);
}


void Object2DSpace::calculateAngle()
{
    tf::Quaternion q(m_pose.orientation.x, m_pose.orientation.y, m_pose.orientation.z, m_pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, theta;
    m.getRPY(roll, pitch, theta);
    m_angle = theta;
}
