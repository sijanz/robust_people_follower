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


#ifndef ROBUST_PEOPLE_FOLLOWER_TRACKING_MODULE_H
#define ROBUST_PEOPLE_FOLLOWER_TRACKING_MODULE_H


#include "person.h"


/**
 * @brief The Tracking Module processes data received from the /body_tracker/skeleton topic and manages a list of
 * tracked persons. In addition to that, the Tracking Module holds information about the current target.
 */
class TrackingModule
{
public:

    /*
     * ********** SPECIAL METHODS **********
     */

    /**
     * @brief Standard constructor. Initializes the target and allocates memory on the heap for the list of tracked
     * persons.
     */
    TrackingModule();


    /*
     * ********** SETTERS **********
     */

    /**
     * @brief Setter for the current target.
     *
     * @return a reference to the target as a lvalue
     */
    inline Person& target() { return m_target; }


    /**
     * @brief Setter for the list of tracked persons.
     *
     * @return a pointer to the list as a lvalue
     */
    inline std::shared_ptr<std::vector<Person>> trackedPersons() { return m_tracked_persons; }


    /*
     * ********** GETTERS **********
     */

    /**
     * @brief Getter for the current target.
     *
     * @return a reference to the target as a rvalue
     */
    inline const Person& target() const { return m_target; }


    /**
     * @brief Getter for the list of tracked persons.
     *
     * @return a pointer to the list as a rvalue
     */
    inline const std::shared_ptr<std::vector<Person>> trackedPersons() const { return m_tracked_persons; }


    /*
     * ********** STANDARD METHODS **********
     */

    /**
     * @brief Takes the skeleton data and sorts it in the list of tracked persons. If the person with the corresponding
     * body-id from the skeleton data is currently not in the list of tracked persons, a new entry is created. If the
     * person is already in the list of currently tracked persons, the information in the list gets updated. If a
     * gesture is recognized, the corresponding tracked person is selected as a target. The target is unset if a gesture
     * is associated with its body id.
     *
     * @param t_skeleton the skeleton information received from the /body_tacker/skeleton topic
     * @param t_robot_pose the pose information of the robot; used to calculate the absolute position of a person
     * @param t_status reference to the robot's status; needs to be changed if a new target is selected
     */
    void processSkeletonData(const body_tracker_msgs::Skeleton& t_skeleton,
                             const geometry_msgs::PoseStamped& t_robot_pose,
                             StatusModule::Status& t_status);


    // TODO: comment
    void checkForTargetLoss(StatusModule::Status& t_status);


    /**
     * @brief Removes entries of persons that are not tracked anymore. An entry gets deleted if the distance and
     * y-deviation is equal to 0.s
     */
    void managePersonList();


private:

    /** @brief Represents the current target */
    Person m_target{};

    /** @brief A list persons currently tracked  */
    std::shared_ptr<std::vector<Person>> m_tracked_persons{};


    static bool checkForValidData(const body_tracker_msgs::Skeleton& t_skeleton);
};


#endif //ROBUST_PEOPLE_FOLLOWER_TRACKING_MODULE_H
