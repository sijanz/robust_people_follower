#ifndef ROBUST_PEOPLE_FOLLOWER_PERSON_H
#define ROBUST_PEOPLE_FOLLOWER_PERSON_H

#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Point32.h>


/**
 * Stores data related to a tracked person.
 */
class Person
{
private:
    bool m_is_tracked;
    bool m_is_target;
    double m_velocity;
    body_tracker_msgs::Skeleton m_skeleton;
    geometry_msgs::Point32 m_absolute_position;
    geometry_msgs::Point32 m_old_absolute_position;
    ros::Time m_gesture_begin;

public:
    explicit Person(const body_tracker_msgs::Skeleton& t_skeleton);
    void printPersonInfo() const;
    void printVerbosePersonInfo() const;
    bool isTracked() const;
    void setTrackingStatus(bool t_is_tracked);
    int getId() const;
    bool isTarget() const;
    void setTarget(bool t_is_target);
    body_tracker_msgs::Skeleton getSkeleton() const;
    void setSkeleton(const body_tracker_msgs::Skeleton& t_skeleton);
    geometry_msgs::Point32 getAbsolutePosition() const;
    void setAbsolutePosition(const geometry_msgs::Point32& t_absolute_position);
    int getGestureBegin() const;
    void setGestureBegin(const ros::Time& t_gesture_begin);
    double getDistance() const;
    double getYDeviation() const;
    void calculateAbsolutePosition(double t_robot_x, double t_robot_y, double t_robot_angle);
    void calculateVelocity(double t_frequency);
    void updateOldPosition();
    double getVelocity() const;
    void setVelocity(double t_velocity);
};


Person::Person(const body_tracker_msgs::Skeleton& t_skeleton)
{
    m_is_tracked = true;
    m_is_target = false;
    m_velocity = 0;
    m_gesture_begin = ros::Time(0);
    m_absolute_position.x = m_absolute_position.y = m_absolute_position.z = 0;
    m_skeleton = t_skeleton;
}


void Person::printPersonInfo() const
{
    ROS_INFO("id: %d, is target: %d, distance: %f, number of gestures: %d",
             m_skeleton.body_id, m_is_target, m_skeleton.centerOfMass.x, m_skeleton.gesture);
}


void Person::printVerbosePersonInfo() const
{
    ROS_INFO("id: %d", m_skeleton.body_id);
    ROS_INFO("  velocity: %f", m_velocity);
    ROS_INFO("  position (relative):");
    ROS_INFO("    x: %f", m_skeleton.joint_position_spine_mid.x);
    ROS_INFO("    y: %f", m_skeleton.joint_position_spine_mid.y);
    ROS_INFO("  position (absolute):");
    ROS_INFO("    x: %f", m_absolute_position.x);
    ROS_INFO("    y: %f", m_absolute_position.y);
    ROS_INFO("  distance: %f", m_skeleton.centerOfMass.x);
    ROS_INFO("  y-deviation of center of mass: %f\n", m_skeleton.centerOfMass.y);
}


bool Person::isTracked() const
{
    return m_is_tracked;
}


void Person::setTrackingStatus(const bool t_is_tracked)
{
    m_is_tracked = t_is_tracked;
}


int Person::getId() const
{
    return m_skeleton.body_id;
}


bool Person::isTarget() const
{
    return m_is_target;
}


void Person::setTarget(bool t_is_target)
{
    m_is_target = t_is_target;
}


body_tracker_msgs::Skeleton Person::getSkeleton() const
{
    return m_skeleton;
}


void Person::setSkeleton(const body_tracker_msgs::Skeleton& t_skeleton)
{
    m_skeleton = t_skeleton;
}


geometry_msgs::Point32 Person::getAbsolutePosition() const
{
    return m_absolute_position;
}


void Person::setAbsolutePosition(const geometry_msgs::Point32& t_absolute_position)
{
    m_absolute_position = t_absolute_position;
}


int Person::getGestureBegin() const
{
    return m_gesture_begin.sec;
}


void Person::setGestureBegin(const ros::Time& t_gesture_begin)
{
    m_gesture_begin = t_gesture_begin;
}


double Person::getDistance() const
{
    return m_skeleton.centerOfMass.x;
}


double Person::getYDeviation() const
{
    return m_skeleton.centerOfMass.y;
}


void Person::calculateAbsolutePosition(double t_robot_x, double t_robot_y, double t_robot_angle)
{
    m_absolute_position.x = t_robot_x + cos(t_robot_angle) * m_skeleton.joint_position_spine_mid.x -
                            sin(t_robot_angle) * m_skeleton.joint_position_spine_mid.y;
    m_absolute_position.y = t_robot_y + sin(t_robot_angle) * m_skeleton.joint_position_spine_mid.x -
                            cos(t_robot_angle) * m_skeleton.joint_position_spine_mid.y;
}


void Person::calculateVelocity(double t_frequency)
{
    m_velocity = sqrt(pow((m_old_absolute_position.x - m_absolute_position.x), 2) +
                      pow((m_old_absolute_position.y - m_absolute_position.y), 2)) / (1 / t_frequency);
}


void Person::updateOldPosition()
{
    m_old_absolute_position = m_absolute_position;
}


double Person::getVelocity() const
{
    return m_velocity;
}

void Person::setVelocity(double t_velocity)
{
    m_velocity = t_velocity;
}


#endif //ROBUST_PEOPLE_FOLLOWER_PERSON_H
