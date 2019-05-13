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

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
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


/**
 * @brief Constructor.
 */
Turtlebot::Turtlebot()
{
    m_status = WAITING;
    m_velocity = 0.0;
    m_pose.position.x = m_pose.position.y = m_pose.position.z = 0.0;
    m_pose.orientation.x = m_pose.orientation.y = m_pose.orientation.z = m_pose.orientation.w = 0.0;
    m_angle = 0.0;
    m_current_linear = m_current_angular = 0.0;
}


/**
 * @brief Prints out information about the roboter.
 */
void Turtlebot::printTurtlebotInfo() const
{
    const char *status_string;
    switch (m_status) {
        case 0:
            status_string = "WAITING";
            break;
        case 1:
            status_string = "FOLLOWING";
            break;
        default:
            status_string = "SEARCHING";
            break;
    }

    ROS_INFO("Turtlebot information:");
    ROS_INFO("  status: [%s]", status_string);
    ROS_INFO("  velocity: %f", m_velocity);
    ROS_INFO("  position:");
    ROS_INFO("    x: %f", m_pose.position.x);
    ROS_INFO("    y: %f", m_pose.position.y);
    ROS_INFO("  theta: %f\n", m_angle);
}


Turtlebot::Status Turtlebot::getStatus() const
{
    return m_status;
}


/**
 * @brief Setter for the status of the roboter.
 * @param t_status the new status
 */
void Turtlebot::setStatus(const Turtlebot::Status t_status)
{
    m_status = t_status;
}


/**
 * @brief Getter for the pose of the robot, consisting of the position and orientation.
 * @return the pose
 */
geometry_msgs::Pose Turtlebot::getPose()
{
    return m_pose;
}


/**
 * @brief Setter for the pose of the robot.
 * @param t_position_x x-coordinate of the position
 * @param t_position_y y-coordinate of the position
 * @param t_orientation_z z-coordinate of the orientation
 * @param t_orientation_w w-coordinate of the orientation
 */
void Turtlebot::setPose(const geometry_msgs::Pose& t_pose)
{
    m_pose = t_pose;
    calculateAngle();
}


/**
 * @brief Updates the old pose of the robot for the calculation of the robot.
 */
void Turtlebot::updateOldPose()
{
    m_old_pose = m_pose;
}


/**
 * @brief Calculates the velocity of the robot.
 * @param frequency the frequency of the main loop, needed to get the time difference
 */
void Turtlebot::calculateVelocity(double t_frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / t_frequency);
}


void Turtlebot::calculateAngle()
{
    tf::Quaternion q(m_pose.orientation.x, m_pose.orientation.y, m_pose.orientation.z, m_pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, theta;
    m.getRPY(roll, pitch, theta);
    m_angle = theta;
}


double Turtlebot::getAngle() const
{
    return m_angle;
}


// TODO: test, make smoother, make threshold variable
geometry_msgs::Twist& Turtlebot::setVelocityCommand(const Person& t_target,
                                                    std::deque<geometry_msgs::PointStamped>& t_goal_list,
                                                    geometry_msgs::Twist& t_msg)
{
    t_msg.linear.x = 0.0;
    t_msg.angular.z = 0.0;

    if (!t_goal_list.empty()) {
        geometry_msgs::PointStamped& current_goal = t_goal_list.at(0);


        double inc_x = current_goal.point.x - m_pose.position.x;
        double inc_y = current_goal.point.y - m_pose.position.y;

        double angle_to_goal = atan2(inc_y, inc_x);
        double distance_to_goal = sqrt(pow(current_goal.point.x - m_pose.position.x, 2)
                                       + pow(current_goal.point.y - m_pose.position.y, 2));

        //if (t_target.getDistance() > 1800) {
        if (distance_to_goal < 0.3) {

            // input 40% less acceleration
            t_msg.linear.x = m_current_linear - (m_current_linear / 100) * 40;
            t_msg.angular.z = m_current_angular - (m_current_angular / 100) * 40;

            t_goal_list.pop_front();

            if (!t_goal_list.empty())
                current_goal = t_goal_list.at(0);


        } else if (angle_to_goal - m_angle > 0.2)
            t_msg.angular.z = 0.5 * (angle_to_goal - m_angle);
        else if (angle_to_goal - m_angle < -0.2)
            t_msg.angular.z = -0.5 * std::abs(angle_to_goal - m_angle);
        else
            t_msg.linear.x = 0.4;
        //t_msg.linear.x = 0.32 * (t_target.getDistance() / 1000) - 0.576;

        // move backwards if target is too near
    } else if (t_target.getDistance() < 1000 && t_target.getDistance() > 0) {
        t_msg.linear.x = 2 * (t_target.getDistance() / 1000) - 2;
    }

    /*// keep human in center if the distance is less than the threshold
else {
    if (t_target.getYDeviation() < -50)
        t_msg.angular.z = -0.0025 * std::abs(t_target.getYDeviation());
    else if (t_target.getYDeviation() > 50)
        t_msg.angular.z = 0.0025 * t_target.getYDeviation();
}*/
    m_current_linear = t_msg.linear.x;
    m_current_angular = t_msg.angular.z;

    /*
    // no goals
} else {
    // angular velocity
    if (t_target.getYDeviation() < -50) {
        t_msg.angular.z = -0.0025 * std::abs(t_target.getYDeviation());
    } else if (t_target.getYDeviation() > 50) {
        t_msg.angular.z = 0.0025 * t_target.getYDeviation();
    } else
        t_msg.angular.z = 0;

    // linear velocity
    if (t_target.getDistance() < 1000 && t_target.getDistance() > 0.0) {
        t_msg.linear.x = 2 * (t_target.getDistance() / 1000) - 2;
    }
}
*/

    return t_msg;

/*
// stop robot if target made gesture
if (m_status == WAITING) {
    t_msg.angular.z = 0.0;
    t_msg.linear.x = 0.0;
} else {

    // angular velocity
    if (t_target.getYDeviation() < -50) {
        t_msg.angular.z = -0.0025 * std::abs(t_target.getYDeviation());
        ROS_INFO("[TURNING LEFT at %f]", t_msg.angular.z);
    } else if (t_target.getYDeviation() > 50) {
        t_msg.angular.z = 0.0025 * t_target.getYDeviation();
        ROS_INFO("[TURNING RIGHT at %f]", t_msg.angular.z);
    } else
        t_msg.angular.z = 0;

    // linear velocity
    if (t_target.getDistance() > 1800) {
        t_msg.linear.x = 0.32 * (t_target.getDistance() / 1000) - 0.576;
        ROS_INFO("[MOVING FORWARD AT %f]", t_msg.linear.x);
    } else if (t_target.getDistance() < 1000 && t_target.getDistance() > 0.0) {
        t_msg.linear.x = 2 * (t_target.getDistance() / 1000) - 2;
        ROS_INFO("[MOVING BACKWARDS AT %f]", t_msg.linear.x);
    }
}
 */

}


#endif //ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H
