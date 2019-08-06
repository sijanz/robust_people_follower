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


#ifndef ROBUST_PEOPLE_FOLLOWER_RECOVERY_MODULE_H
#define ROBUST_PEOPLE_FOLLOWER_RECOVERY_MODULE_H


#include <geometry_msgs/Point32.h>

#include "person.h"
#include "status_module.h"


/**
 * @brief Predicts the target's position if the line of sight to the target is lost. Receives target information
 * from the tracking module. The target's predicted position is calculated using the Constant Turn Rate and Acceleration
 * motion model. Along with the calculation of the predicted position, the Recovery Module calculates a radius in which
 * the target is likely to be located. If a tracked person is inside the radius, it is set as the new target.
 */
class RecoveryModule
{
public:

    /*
     * ********** SPECIAL METHODS **********
     */

    /**
     * @brief Default constructor, initializes the predicted positions and the prediction radius to 0 and the stop flag
     * to false.
     */
    inline RecoveryModule() = default;


    /*
     * ********** GETTERS **********
     */

    /**
     * Getter for the predicted target position.
     *
     * @return the predicted target position as a rvalue
     */
    inline const geometry_msgs::Point32 predictedTargetPosition() const { return m_predicted_target_position; }


    /**
     * Getter for the prediction radius.
     *
     * @return the prediction radius as a rvalue
     */
    inline const double predictionRadius() const { return m_prediction_radius; }

    inline const double predictedVelocity() const { return m_predicted_velocity; }


    /*
     * ********** STANDARD METHODS **********
     */
    void predictTargetPosition(const Person& t_target, double t_interval_sec);
    void reIdentify(Person& t_target, const std::shared_ptr<std::vector<Person>>& t_tracked_persons,
                    const std::shared_ptr<std::deque<geometry_msgs::PointStamped>>& t_waypoint_list,
                    StatusModule::Status& t_status);

private:
    geometry_msgs::Point32 m_predicted_target_position{};
    geometry_msgs::Point32 m_old_predicted_target_position{};
    bool m_prediction_stop{};
    double m_prediction_radius{};
    double m_predicted_velocity{};
};


#endif //ROBUST_PEOPLE_FOLLOWER_RECOVERY_MODULE_H
