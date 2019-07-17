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


/**
 * @brief Represents a base class for objects in a 2-dimensional Euclidean space. Holds information for the position,
 * angle and velocity of the object and provides getters and setters.
 */
class Object2DSpace
{
public:

    /*
     * ********** PURE VIRTUAL METHODS **********
     */

    /**
     * @brief Prints out information about the object. Look at the header file of the class that inherits from this base
     * class to find detailed documentation.
     */
    virtual void printInfo() const = 0;


    /**
     * @brief Calculates the angle of the object. Look at the header file of the class that inherits from this base
     * class to find detailed documentation.
     */
    virtual void calculateAngle() = 0;


    /**
     * @brief Calculates the velocity of the object in m/s. Look at the header file of the class that inherits from this
     * base class to find detailed documentation.
     *
     * @param t_frequency the frequency of the main loop use for calculation of the velocity
     */
    virtual void calculateVelocity(double t_frequency) = 0;


    /*
     * ********** SETTERS **********
     */

    /**
     * @brief Setter for the m_pose-member.
     *
     * @return pose information as a lvalue
     */
    inline geometry_msgs::Pose& pose() { return m_pose; }


    /*
     * ********** GETTERS **********
     */

    /**
     * @brief Getter for the m_pose member.
     *
     * @return pose information as a rvalue
     */
    inline const geometry_msgs::Pose& pose() const { return m_pose; }


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
    inline const double angle() const { return m_angle; }


    /*
     * ********** STANDARD METHODS **********
     */

    /**
     * @brief Sets the old pose equal to the new pose. Is needed in order to calculate the velocity and angle.
     */
    inline void updatePose() { m_old_pose = m_pose; }


protected:

    /*
     * ********** MEMBER VARIABLES **********
     */

    /** @brief The current pose (position plus orientation) of the object in 2D-space. */
    geometry_msgs::Pose m_pose{};

    /** @brief The pose at t - 1 of the object in 2D-space. */
    geometry_msgs::Pose m_old_pose{};

    /** @brief The velocity of the object in m/s. */
    double m_velocity{};

    /** @brief The angle of the object in reference to the root of the coordinate system in radians. */
    double m_angle{};
};


#endif //ROBUST_PEOPLE_FOLLOWER_OBJECT_2D_SPACE_H
