#ifndef ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H
#define ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include "person.h"


#define WAITING 0
#define FOLLOWING 1
#define SEARCHING 2


/**
 * Stores data related to the roboter.
 */
class Turtlebot
{
private:
    int m_status;
    double m_velocity;
    geometry_msgs::Pose m_pose;
    geometry_msgs::Pose m_old_pose;
    double m_angle;

    void calculateAngle();

public:
    Turtlebot();
    void printTurtlebotInfo() const;
    void setStatus(int t_status);
    double getVelocity() const;
    geometry_msgs::Pose getPose();
    void setPose(const geometry_msgs::Pose& t_pose);
    void updateOldPose();
    void calculateVelocity(double t_frequency);
    double getAngle() const;
};


/**
 * @brief Constructor.
 */
Turtlebot::Turtlebot()
{
    m_status = WAITING;
    m_velocity = 0;
    m_pose.position.x = m_pose.position.y = m_pose.position.z = 0;
    m_pose.orientation.x = m_pose.orientation.y = m_pose.orientation.z = m_pose.orientation.w = 0;
    m_angle = 0;
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
    ROS_INFO("  orientation:");
    ROS_INFO("    z: %f", m_pose.orientation.z);
    ROS_INFO("    w: %f", m_pose.orientation.w);
    ROS_INFO("  angle: %f\n", m_angle);
}


/**
 * @brief Setter for the status of the roboter.
 * @param t_status the new status
 */
void Turtlebot::setStatus(const int t_status)
{
    m_status = t_status;
}


/**
 * @brief Getter for the velocity of the robot.
 * @return the velocity
 */
double Turtlebot::getVelocity() const
{
    return m_velocity;
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
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    m_angle = yaw;
}


double Turtlebot::getAngle() const
{
    return m_angle;
}


#endif //ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H
