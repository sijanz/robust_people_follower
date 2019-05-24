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


struct QuaternionStamped
{
    tf::Quaternion quaternion{};
    ros::Time stamp{};

    inline QuaternionStamped() = default;
    inline QuaternionStamped(double t_yaw, const ros::Time& t_stamp) :
            quaternion{}, stamp(t_stamp) { quaternion.setEuler(t_yaw, 0.0, 0.0); }
    inline QuaternionStamped& operator=(const QuaternionStamped& rhs) = default;
    inline QuaternionStamped& operator+(const QuaternionStamped& rhs) { quaternion += rhs.quaternion; }

    inline QuaternionStamped& operator+=(const QuaternionStamped& rhs)
    {
        quaternion += rhs.quaternion;
        return *this;
    }

    friend inline QuaternionStamped& operator/=(QuaternionStamped& lhs, const size_t t_size)
    {
        lhs.quaternion.setX(lhs.quaternion.x() / t_size);
        lhs.quaternion.setY(lhs.quaternion.y() / t_size);
        lhs.quaternion.setZ(lhs.quaternion.z() / t_size);
        lhs.quaternion.setW(lhs.quaternion.w() / t_size);
        return lhs;
    }
};


#endif //ROBUST_PEOPLE_FOLLOWER_STAMPED_TYPES_H
