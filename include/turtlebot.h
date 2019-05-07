#ifndef ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H
#define ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H

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

public:
    Turtlebot();
    void printTurtlebotInfo() const;
    int getStatus();
    void setStatus(int t_status);
    double getVelocity() const;
    geometry_msgs::Pose getPose();
    void setPose(double t_position_x, double t_position_y, double t_orientation_z, double t_orientation_w);
    void updateOldPose();
    void calculateVelocity(double t_frequency);
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
}


/**
 * @brief Prints out information about the roboter.
 */
void Turtlebot::printTurtlebotInfo() const
{
    ROS_INFO("Turtlebot information:");
    ROS_INFO("  velocity: %f", m_velocity);
    ROS_INFO("  position:");
    ROS_INFO("    x: %f", m_pose.position.x);
    ROS_INFO("    y: %f", m_pose.position.y);
    ROS_INFO("  orientation:");
    ROS_INFO("    z: %f", m_pose.orientation.z);
    ROS_INFO("    w: %f\n", m_pose.orientation.w);
}


/**
 * @brief Getter for the status of the roboter.
 * @return the status ({WAITING, FOLLOWING, SEARCHING})
 */
int Turtlebot::getStatus()
{
    return m_status;
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
void Turtlebot::setPose(const double t_position_x, const double t_position_y, const double t_orientation_z,
                        const double t_orientation_w)
{
    m_pose.position.x = t_position_x;
    m_pose.position.y = t_position_y;
    m_pose.orientation.z = t_orientation_z;
    m_pose.orientation.w = t_orientation_w;
}


/**
 * @brief Updates the old pose of the robot for the calculation of the robot.
 */
void Turtlebot::updateOldPose()
{
    m_old_pose.position.x = m_pose.position.x;
    m_old_pose.position.y = m_pose.position.y;
    m_old_pose.orientation.z = m_pose.orientation.z;
    m_old_pose.orientation.w = m_pose.orientation.w;
}


/**
 * @brief Calculates the velocity of the robot.
 * @param frequency the frequency of the main loop, needed to get the time difference
 */
void Turtlebot::calculateVelocity(double frequency)
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) +
                      pow((m_old_pose.position.y - m_pose.position.y), 2)) / (1 / frequency);
}


#endif //ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H