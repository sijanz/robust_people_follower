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


#ifndef ROBUST_PEOPLE_FOLLOWER_STATUS_MODULE_H
#define ROBUST_PEOPLE_FOLLOWER_STATUS_MODULE_H

#include "object_2d_space.h"


/**
 * @brief The Status Module manages information about the current state of the robot. It holds the robot's status used
 * to determine the robot's actions as well as its position, orientation and velocity. The Status Module receives its
 * data from the /odom topic and provides its information to the other modules.
 */
class StatusModule : public Object2DSpace
{
public:

    /**
     * @brief Represents the status of the robot.
     */
    enum Status : unsigned char
    {

        /** @brief The robots has a WAITING status if no target is selected. */
                WAITING = 0,

        /** @brief Is set after the "follow-me"-gesture is recognized; the robot follows the target depending on its
         * distance to the robot. */
                FOLLOWING = 1,

        /** @brief The robot lost the line of sight to the target but still has waypoints to drive to. */
                LOS_LOST = 2,

        /** @brief The line of sight to the target is lost and no waypoints are left to drive to. */
                SEARCHING = 3
    };


    /*
     * ********** SPECIAL METHODS **********
     */

    /**
     * @brief Standard constructor. Initializes the status as well as the pose and velocity with their default values.
     */
    StatusModule() = default;


    /*
     * ********** INHERITED METHODS **********
     */

    /**
     * @brief Prints out information about the robot. The information contains of the robot's status, velocity in m/s,
     * position in the coordinate system as x-y-value and orientation in radians.
     */
    void printInfo() const override;


    /*
     * ********** SETTERS **********
     */

    /**
     * @brief Setter for the status of the robot.
     *
     * @return the status of the robot as a lvalue
     */
    inline StatusModule::Status& status() { return m_status; }


    /*
     * ********** GETTERS **********
     */

    /**
     * @brief Getter for th status of the robot.
     *
     * @return the status of the robot as a rvalue
     */
    inline const StatusModule::Status status() const { return m_status; }


    /*
     * ********** STANDARD METHODS **********
     */

    /**
     * @brief Sets the data extracted from the /odom topic to the members. It first sets the pose, before calculating
     * the robot's angle and velocity. Lastly the robot's old pose is updated.
     *
     * @param t_pose_stamped pose information extracted from the /odom topic
     */
    void processOdometryData(const geometry_msgs::PoseStamped& t_pose_stamped);


private:

    /** @brief Stores the current status of the robot in form of the Status enumeration. */
    Status m_status{};
};


#endif //ROBUST_PEOPLE_FOLLOWER_STATUS_MODULE_H
