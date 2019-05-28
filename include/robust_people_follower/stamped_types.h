#ifndef ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H
#define ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H


#include <tf/LinearMath/Quaternion.h>


struct VelocityStamped
{
    // member variables
    double velocity{};
    ros::Time stamp{};

    // constructors
    inline VelocityStamped() = default;
    inline VelocityStamped(double t_velocity, const ros::Time& t_stamp) : velocity(t_velocity), stamp(t_stamp) {}

    // overloaded operators
    inline VelocityStamped& operator=(const VelocityStamped& rhs) = default;

    inline VelocityStamped& operator+=(const VelocityStamped& rhs)
    {
        velocity += rhs.velocity;
        return *this;
    }

    inline VelocityStamped& operator/=(const size_t t_size)
    {
        velocity /= t_size;
        return *this;
    }
};


struct AngleStamped
{
    // member variables
    double angle{};
    ros::Time stamp{};

    // constructors
    AngleStamped(const double t_angle, const ros::Time& t_stamp) : angle{t_angle}, stamp{t_stamp} {}

    // overloaded operators
    friend inline double operator+=(double& lhs, const AngleStamped& rhs)
    {
        lhs += rhs.angle;
        return lhs;
    }
};


#endif //ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H
