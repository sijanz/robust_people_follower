#ifndef ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H
#define ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H

#include <geometry_msgs/Pose.h>
#include "person.h"


#define WAITING 0
#define FOLLOWING 1
#define SEARCHING 2


/**l
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
    void setOldPose();
    void calculateVelocity();
};


Turtlebot::Turtlebot()
{
    m_status = WAITING;
    m_velocity = 0;
    m_pose.position.x = m_pose.position.y = m_pose.position.z = 0;
    m_pose.orientation.x = m_pose.orientation.y = m_pose.orientation.z = m_pose.orientation.w = 0;
}


void Turtlebot::printTurtlebotInfo() const
{
    ROS_INFO("Turtlebot information:");
    ROS_INFO("  velocity: %f", m_velocity);
    ROS_INFO("  position (old):");
    ROS_INFO("    x: %f", m_old_pose.position.x);
    ROS_INFO("    y: %f", m_old_pose.position.y);
    ROS_INFO("  position:");
    ROS_INFO("    x: %f", m_pose.position.x);
    ROS_INFO("    y: %f", m_pose.position.y);
    ROS_INFO("  orientation:");
    ROS_INFO("    z: %f", m_pose.orientation.z);
    ROS_INFO("    w: %f\n", m_pose.orientation.w);
}


int Turtlebot::getStatus()
{
    return m_status;
}


void Turtlebot::setStatus(const int t_status)
{
    m_status = t_status;
}

double Turtlebot::getVelocity() const
{
    return m_velocity;
}


geometry_msgs::Pose Turtlebot::getPose()
{
    return m_pose;
}


void Turtlebot::setPose(const double t_positionX, const double t_positionY, const double t_orientationZ,
                        const double t_orientationW)
{
    m_pose.position.x = t_positionX;
    m_pose.position.y = t_positionY;
    m_pose.orientation.z = t_orientationZ;
    m_pose.orientation.w = t_orientationW;
}


void Turtlebot::setOldPose()
{
    m_old_pose.position.x = m_pose.position.x;
    m_old_pose.position.y = m_pose.position.y;
    m_old_pose.orientation.z = m_pose.orientation.z;
    m_old_pose.orientation.w = m_pose.orientation.w;
}

void Turtlebot::calculateVelocity()
{
    m_velocity = sqrt(pow((m_old_pose.position.x - m_pose.position.x), 2) + pow((m_old_pose.position.y - m_pose.position.y), 2)) / 0.1;
}


#endif //ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_H
