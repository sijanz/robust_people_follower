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


#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include "robust_people_follower/stamped_types.h"


/**
 * @brief Represents a base class for objects in a 2-dimensional Euclidean space. Holds information for the position,
 * angle and velocity of the object and provides getters and setters.
 */
class Object2DSpace
{
public:

    /*
     * ********** GENERAL PURPOSE METHODS **********
     */
    static double yawFromPose(const geometry_msgs::PoseStamped& t_pose)
    {
        auto q{tf::Quaternion{t_pose.pose.orientation.x, t_pose.pose.orientation.y, t_pose.pose.orientation.z,
                              t_pose.pose.orientation.w}};
        auto m{tf::Matrix3x3{q}};

        auto roll{0.0}, pitch{0.0}, theta{0.0};
        m.getRPY(roll, pitch, theta);

        return theta;
    }


    /*
     * ********** PURE VIRTUAL METHODS **********
     */

    /**
     * @brief Prints out information about the object. Look at the header file of the class that inherits from this base
     * class to find detailed documentation.
     */
    virtual void printInfo() const = 0;


    /*
     * ********** SETTERS **********
     */

    /**
     * @brief Setter for the m_pose-member.
     *
     * @return pose information as a lvalue
     */
    inline geometry_msgs::PoseStamped& pose() { return m_current_pose; }


    inline bool& valuesSet() { return m_values_set; }


    /*
     * ********** GETTERS **********
     */

    /**
     * @brief Getter for the m_pose member.
     *
     * @return pose information as a rvalue
     */
    inline const geometry_msgs::PoseStamped& pose() const { return m_current_pose; }


    /**
     * @brief Getter for velocity information.
     *
     * @return velocity in m/s as a rvalue
     */
    inline const double velocity() const { return m_velocity; }


    /**
     * @brief Getter for the angle.
     *
     * @return angle in radians as a rvalue
     */
    inline const double angle() const { return yawFromPose(m_current_pose); }


    /*
     * ********** STANDARD METHODS **********
     */

    /**
     * @brief Calculates the velocity of the object in m/s by dividing the Euclidean distance between the current pose
     * and the last pose by the elapsed time.
     */
    void calculateVelocity()
    {

        // if the current or old position is not specified, the velocity is set to 0
        if (m_current_pose.pose.position.x == 0.0 || m_old_pose.pose.position.x == 0.0)
            m_velocity = 0.0;

            // otherwise, the velocity is calculated by the distance between the last two points divided by the difference
            // in time
        else {
            auto delta_t{(m_current_pose.header.stamp - m_old_pose.header.stamp).toSec()};
            m_velocity = sqrt(pow((m_old_pose.pose.position.x - m_current_pose.pose.position.x), 2) +
                              pow((m_old_pose.pose.position.y - m_current_pose.pose.position.y), 2)) / delta_t;
        }
    }


    /**
     * @brief Sets the old pose equal to the new pose. Is needed in order to calculate the velocity and angle.
     */
    inline void updatePose() { m_old_pose = m_current_pose; }


protected:

    /*
     * ********** MEMBER VARIABLES **********
     */

    /** @brief The current pose (position plus orientation) of the object in 2D-space. */
    geometry_msgs::PoseStamped m_current_pose{};

    /** @brief The pose at t - 1 of the object in 2D-space. */
    geometry_msgs::PoseStamped m_old_pose{};

    /** @brief The velocity of the object in m/s. */
    double m_velocity{};

    bool m_values_set{false};
};


#endif //ROBUST_PEOPLE_FOLLOWER_OBJECT_2D_SPACE_H
