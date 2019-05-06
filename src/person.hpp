#ifndef ROBUST_PEOPLE_FOLLOWER_PERSON_HPP
#define ROBUST_PEOPLE_FOLLOWER_PERSON_HPP

#include "structs.hpp"


/**
 * Stores data related to a tracked person.
 */
class Person
{
public:
    bool isTracked;
    bool isTarget;
    SkeletonInfo skeleton;
    Position2D position;

    // TODO: calculate velocity
    double velocity;

    ros::Time gestureBegin;


    Person(SkeletonInfo skeleton, Position2D position)
    {
        isTracked = true;
        isTarget = false;
        this->skeleton = skeleton;
        this->position = position;
        velocity = 0;
        gestureBegin = ros::Time(0);
    }
};


#endif //ROBUST_PEOPLE_FOLLOWER_PERSON_HPP
