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
    geometry_msgs::Point32 m_absolutePosition;
    ros::Time m_gesture_begin;

public:
    explicit Person(const body_tracker_msgs::Skeleton& t_skeleton);
    void printPersonInfo() const;
    void printVolatilePersonInfo() const;
    bool isTracked() const;
    void setTrackingStatus(bool t_is_tracked);
    int getId() const;
    bool isTarget() const;
    void setTarget(bool t_is_target);
    void setSkeleton(const body_tracker_msgs::Skeleton& t_skeleton);
    int getGestureBegin() const;
    void setGestureBegin(const ros::Time& t_gesture_begin);
    double getDistance() const;
    double getYDeviation() const;
    void calculateVelocity();
};


Person::Person(const body_tracker_msgs::Skeleton& t_skeleton)
{
    m_is_tracked = true;
    m_is_target = false;
    m_velocity = 0;
    m_gesture_begin = ros::Time(0);
    m_absolutePosition.x = m_absolutePosition.y = m_absolutePosition.z = 0;
    m_skeleton = t_skeleton;
}


void Person::printPersonInfo() const
{
    ROS_INFO("id: %d, is taget: %d, distance: %f, number of gestures: %d",
             m_skeleton.body_id, m_is_target, m_skeleton.centerOfMass.x, m_skeleton.gesture);
}


void Person::printVolatilePersonInfo() const
{
    ROS_INFO("id: %d", m_skeleton.body_id);
    ROS_INFO("  is target: %d", m_is_target);
    ROS_INFO("  position (relative):");
    ROS_INFO("    x: %f", m_skeleton.joint_position_spine_mid.x);
    ROS_INFO("    y: %f", m_skeleton.joint_position_spine_mid.y);

    // TODO: calculate correct absolute position
    ROS_INFO("  position (absolute):");
    ROS_INFO("    x: %f", m_absolutePosition.x);
    ROS_INFO("    y: %f", m_absolutePosition.y);

    ROS_INFO("  number of gestures: %d", m_skeleton.gesture);
    ROS_INFO("  gesture begin time: %d", m_gesture_begin.sec);
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


void Person::setSkeleton(const body_tracker_msgs::Skeleton& t_skeleton)
{
    m_skeleton = t_skeleton;
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


void Person::calculateVelocity()
{
    // TODO
}


#endif //ROBUST_PEOPLE_FOLLOWER_PERSON_H
