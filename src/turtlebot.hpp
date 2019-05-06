#ifndef ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_HPP
#define ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_HPP

#include "structs.hpp"


#define WAITING 0
#define FOLLOWING 1
#define SEARCHING 2


/**
 * Stores data related to the roboter.
 */
class Turtlebot
{
public:
    int status;
    Position2D position;
    Orientation orientation;


    Turtlebot()
    {
        status = WAITING;
        position.x = position.y = 0;
        orientation.z = orientation.w = 0;
    }
};


#endif //ROBUST_PEOPLE_FOLLOWER_TURTLEBOT_HPP
