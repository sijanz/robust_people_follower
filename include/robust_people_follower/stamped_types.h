#ifndef ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H
#define ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H


#include <tf/LinearMath/Quaternion.h>


/**
 * @brief This structure represents a velocity (in m/s) in combination with a timestamp.
 */
struct VelocityStamped
{
    /*
     * ********** MEMBER VARIABLES **********
     */

    /** @brief The velocity in m/s. */
    double velocity{};

    /** @brief The timestamp in ROS-time. */
    ros::Time stamp{};


    /*
     * ********** CONSTRUCTORS **********
     */

    /**
     * @brief Default constructor, initializes the velocity and the timestamp to 0.
     */
    inline VelocityStamped() = default;


    /**
     * @brief Alternative constructor, initializes the member according to the parameters.
     *
     * @param t_velocity the velocity to set in m/s
     * @param t_stamp the timestamp to set in ROS-time
     */
    inline VelocityStamped(double t_velocity, const ros::Time& t_stamp) : velocity{t_velocity}, stamp{t_stamp} {}


    /*
     * ********** OVERLOADED OPERATORS **********
     */

    /**
     * @brief Overloaded "="-operator. Sets the members accordingly to the parameter.
     *
     * @param rhs the VelocityStamped-object to get the members from
     * @return itself
     */
    inline VelocityStamped& operator=(const VelocityStamped& rhs) = default;


    /**
     * @brief Overloaded "+="-operator. Adds up the velocity from the parameterized object.
     *
     * @param rhs the VelocityStamped-object to get the velocity from
     * @return itself
     */
    inline VelocityStamped& operator+=(const VelocityStamped& rhs)
    {
        velocity += rhs.velocity;
        return *this;
    }
};


/**
 * @brief This structure represents an angle (in radians) in combination with a timestamp.
 */
struct AngleStamped
{
    /*
     * ********** MEMBER VARIABLES **********
     */

    /** @brief The angle in radians. */
    double angle{};

    /** @brief The timestamp in ROS-time. */
    ros::Time stamp{};


    /*
     * ********** CONSTRUCTORS **********
     */

    /**
    * @brief Default constructor, initializes the angle and the timestamp to 0.
    */
    AngleStamped() = default;


    /**
     * @brief Alternative constructor, initializes the member according to the parameters.
     *
     * @param t_angle the angle to set in radians
     * @param t_stamp the timestamp to set in ROS-time
     */
    AngleStamped(const double t_angle, const ros::Time& t_stamp) : angle{t_angle}, stamp{t_stamp} {}
};


#endif //ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H
