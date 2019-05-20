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

#include "robust_people_follower/robot.h"


// TODO: use initializer list
Robot::Robot()
{
    m_status = WAITING;
    m_velocity = 0.0;
    m_pose.position.x = m_pose.position.y = m_pose.position.z = 0.0;
    m_pose.orientation.x = m_pose.orientation.y = m_pose.orientation.z = m_pose.orientation.w = 0.0;
    m_angle = 0.0;
    m_current_linear = m_current_angular = 0.0;
}


void Robot::printInfo() const
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

    ROS_INFO("Robot information:");
    ROS_INFO("  status: [%s]", status_string);
    ROS_INFO("  velocity: %f", m_velocity);
    ROS_INFO("  position:");
    ROS_INFO("    x: %f", m_pose.position.x);
    ROS_INFO("    y: %f", m_pose.position.y);
    ROS_INFO("  theta: %f\n", m_angle);
}


Robot::Status Robot::getStatus() const
{
    return m_status;
}


void Robot::setStatus(const Robot::Status t_status)
{
    m_status = t_status;
}


// TODO: test, make threshold variable
geometry_msgs::Twist Robot::setVelocityCommand(const Person& t_target,
                                               std::deque<geometry_msgs::PointStamped>& t_goal_list)
{
    if (t_target.getDistance() < 1800)
        t_goal_list.clear();

    geometry_msgs::Twist speed;

    speed.linear.x = 0.0;
    speed.angular.z = 0.0;

    if (m_status == Status::FOLLOWING || m_status == Status::SEARCHING) {
        if (!t_goal_list.empty()) {

            // move backwards if target is too near
            if (t_target.getDistance() < 1000 && t_target.getDistance() > 0)
                speed.linear.x = 2 * (t_target.getDistance() / 1000) - 2;
            else if (t_target.getDistance() > 1000 && t_target.getDistance() < 1800) {
                if (t_target.getYDeviation() < -50) {
                    speed.angular.z = -0.0025 * std::abs(t_target.getYDeviation());
                    ROS_INFO("[TURNING LEFT at %f]", speed.angular.z);
                } else if (t_target.getYDeviation() > 50) {
                    speed.angular.z = 0.0025 * t_target.getYDeviation();
                    ROS_INFO("[TURNING RIGHT at %f]", speed.angular.z);
                }
            } else if (t_target.getDistance() == 0 || t_target.getDistance() > 1800) {
                geometry_msgs::PointStamped& current_goal = t_goal_list.at(0);


                double inc_x = current_goal.point.x - m_pose.position.x;
                double inc_y = current_goal.point.y - m_pose.position.y;

                double angle_to_goal = atan2(inc_y, inc_x);
                double distance_to_goal = sqrt(pow(current_goal.point.x - m_pose.position.x, 2)
                                               + pow(current_goal.point.y - m_pose.position.y, 2));

                // roboter has reached the goal
                if (distance_to_goal < 0.3) {

                    // input 20% less acceleration
                    speed.linear.x = m_current_linear - (m_current_linear / 100) * 20;
                    speed.angular.z = m_current_angular - (m_current_angular / 100) * 20;

                    t_goal_list.pop_front();

                    if (!t_goal_list.empty())
                        current_goal = t_goal_list.at(0);

                    // FIXME: threshold must be smaller to negate pendulum effect
                } else if (angle_to_goal - m_angle > 0.2)
                    //speed.angular.z = 1.0;
                    speed.angular.z = 0.5 * (angle_to_goal - m_angle);
                else if (angle_to_goal - m_angle < -0.2)
                    //speed.angular.z = 1.0;
                    speed.angular.z = -0.5 * std::abs(angle_to_goal - m_angle);
                //speed.linear.x = 0.32 * (t_target.getDistance() / 1000) - 0.576;
                speed.linear.x = 0.3;

            }
        }

            // no goals
        else {

            // angular velocity
            if (t_target.getYDeviation() < -50) {
                speed.angular.z = -0.0025 * std::abs(t_target.getYDeviation());
            } else if (t_target.getYDeviation() > 50) {
                speed.angular.z = 0.0025 * t_target.getYDeviation();
            } else
                speed.angular.z = 0;

            // linear velocity
            if (t_target.getDistance() < 1000 && t_target.getDistance() > 0.0) {
                speed.linear.x = 2 * (t_target.getDistance() / 1000) - 2;
            }
        }
    }

    m_current_linear = speed.linear.x;
    m_current_angular = speed.angular.z;

    /*
    // FIXME: robot is stationary only
    speed.linear.x = 0.0;

    // angular velocity
    if (t_target.getYDeviation() < -50) {
        speed.angular.z = -0.0025 * std::abs(t_target.getYDeviation());
    } else if (t_target.getYDeviation() > 50) {
        speed.angular.z = 0.0025 * t_target.getYDeviation();
    } else
        speed.angular.z = 0;
        */

    return speed;
}


void Robot::calculateAngle()
{

    tf::Quaternion q(m_pose.orientation.x, m_pose.orientation.y, m_pose.orientation.z, m_pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, theta;
    m.getRPY(roll, pitch, theta);

    m_angle = theta;
}
