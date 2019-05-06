#ifndef ROBUST_PEOPLE_FOLLOWER_STRUCTS_HPP
#define ROBUST_PEOPLE_FOLLOWER_STRUCTS_HPP


/**
 * Stores a position in 2-dimensional space.
 */
struct Position2D
{
    double x;
    double y;
};


/**
 * Stores an orientation in 2-dimensional space.
 */
struct Orientation
{
    double z;
    double w;
};


/**
 * Stores data related to the skeleton info of a person.
 */
struct SkeletonInfo
{
    int bodyId;
    int numberOfGestures;
    float distanceCenterOfMass;
    float centerOfMassY;
};

#endif //ROBUST_PEOPLE_FOLLOWER_STRUCTS_HPP
