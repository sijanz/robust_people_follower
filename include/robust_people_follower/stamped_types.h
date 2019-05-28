#ifndef ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H
#define ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H


#include <tf/LinearMath/Quaternion.h>


struct VelocityStamped
{
    double velocity{};
    ros::Time stamp{};

    inline VelocityStamped() = default;
    inline VelocityStamped(double t_velocity, const ros::Time& t_stamp) : velocity(t_velocity), stamp(t_stamp) {}
    inline VelocityStamped& operator=(const VelocityStamped& rhs) = default;
    inline VelocityStamped& operator+(const VelocityStamped& rhs) { velocity += rhs.velocity; }

    inline VelocityStamped& operator+=(const VelocityStamped& rhs)
    {
        velocity += rhs.velocity;
        return *this;
    }

    friend inline VelocityStamped& operator/=(VelocityStamped& lhs, const size_t t_size)
    {
        lhs.velocity /= t_size;
        return lhs;
    }
};


struct AngleStamped
{
    double angle{};
    ros::Time stamp{};

    AngleStamped(const double t_angle, const ros::Time& t_stamp) : angle{t_angle}, stamp{t_stamp} {}

    friend inline double operator+=(double& lhs, const AngleStamped& rhs)
    {
        lhs += rhs.angle;
        return lhs;
    }
};


#endif //ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H
