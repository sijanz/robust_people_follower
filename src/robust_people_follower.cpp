#include <stdlib.h>
#include <cmath>
#include <deque>

#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include "turtlebot.h"


class RobustPeopleFollower
{
private:

    // publisher
    ros::Publisher m_velocity_command_pub;
    ros::Publisher m_robot_velocity_pub;
    ros::Publisher m_target_velocity_pub;
    ros::Publisher m_robot_path_pub;
    ros::Publisher m_target_path_pub;

    // instance of the turtlebot robot
    Turtlebot m_turtlebot;

    // list of persons in the frame
    std::vector<Person> m_tracked_persons;

    // stores the path of the robot
    nav_msgs::Path m_robot_path;

    // stores the path of the target
    nav_msgs::Path m_target_path;

    // sequence number for robot path
    uint32_t m_seq_robot;

    // sequence number for target path
    uint32_t m_seq_target;

    // object that holds target information
    Person m_target;

    // list that holds goals, at maximum 10
    std::deque<geometry_msgs::Point32> m_goal_list;

    int m_last_goal_time;

    // frequency of the main loop
    const double FREQUENCY = 10.0;

    void debugPrintout();

public:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber skeleton_sub;

    RobustPeopleFollower();
    ~RobustPeopleFollower();
    void runLoop();
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg);
};


// ####### ENTRY POINT #######

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robust_people_follower");
    RobustPeopleFollower robust_people_follower;
    robust_people_follower.odom_sub = robust_people_follower.nh.subscribe("/odom", 10,
                                                                          &RobustPeopleFollower::odometryCallback,
                                                                          &robust_people_follower);
    robust_people_follower.skeleton_sub = robust_people_follower.nh.subscribe("/body_tracker/skeleton", 10,
                                                                              &RobustPeopleFollower::skeletonCallback,
                                                                              &robust_people_follower);
    robust_people_follower.runLoop();

    return 0;
}


// ####### METHOD DEFINITIONS #######

RobustPeopleFollower::RobustPeopleFollower()
{
    m_velocity_command_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    m_robot_velocity_pub = nh.advertise<std_msgs::Float32>("robust_people_follower/robot_velocity", 1000);
    m_target_velocity_pub = nh.advertise<std_msgs::Float32>("robust_people_follower/target_velocity", 1000);
    m_robot_path_pub = nh.advertise<nav_msgs::Path>("robust_people_follower/robot_path", 1000);
    m_target_path_pub = nh.advertise<nav_msgs::Path>("robust_people_follower/target_path", 1000);
    m_turtlebot = {};
    m_tracked_persons = {};
    m_robot_path = m_target_path = {};
    m_seq_robot = m_seq_target = {};
    m_goal_list = {};
    m_last_goal_time = 0;
}


RobustPeopleFollower::~RobustPeopleFollower()
{
    ROS_INFO("robust_people_follower shutting down");
    odom_sub.shutdown();
    skeleton_sub.shutdown();
}


void RobustPeopleFollower::runLoop()
{
    ros::Rate loop_rate(FREQUENCY);

    while (ros::ok()) {

        // processes callbacks
        ros::spinOnce();

        m_turtlebot.calculateVelocity(FREQUENCY);

        // set variables for target
        for (auto& p : m_tracked_persons) {
            if (p.isTarget()) {
                m_target.setSkeleton(p.getSkeleton());
                m_target.setAbsolutePosition(p.getAbsolutePosition());
                m_target.calculateVelocity(FREQUENCY);
                break;
            }
        }

        // add new goal every second
        if (m_target.getDistance() != 0.0 && m_last_goal_time != ros::Time::now().sec) {
            m_goal_list.emplace_back(m_target.getAbsolutePosition());
            m_last_goal_time = ros::Time::now().sec;
        }
        if (m_goal_list.size() > 10)
            m_goal_list.pop_front();

        // TODO: implement actual searching
        // robot loses target
        if (m_target.getDistance() == 0.0 && m_target.getYDeviation() == 0.0) {
            m_turtlebot.setStatus(SEARCHING);
            m_target.setVelocity(0.0);
        }

        // set pose for target
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.seq = m_seq_target;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = m_target.getAbsolutePosition().x;
        pose_stamped.pose.position.y = m_target.getAbsolutePosition().y;
        pose_stamped.pose.position.z = m_target.getAbsolutePosition().z;
        m_target_path.poses.push_back(pose_stamped);

        debugPrintout();

        // move the robot
        // message to store velocity commands in
        geometry_msgs::Twist msg;

        // rotation
        if (m_target.getYDeviation() < -50) {
            msg.angular.z = -0.0025 * std::abs(m_target.getYDeviation());
            ROS_INFO("[TURNING LEFT at %f]", msg.angular.z);
        } else if (m_target.getYDeviation() > 50) {
            msg.angular.z = 0.0025 * m_target.getYDeviation();
            ROS_INFO("[TURNING RIGHT at %f]", msg.angular.z);
        } else
            msg.angular.z = 0;

        // moving straight
        if (m_target.getDistance() > 1800) {
            msg.linear.x = 0.32 * (m_target.getDistance() / 1000) - 0.576;
            ROS_INFO("[MOVING FORWARD AT %f]", msg.linear.x);
        } else if (m_target.getDistance() < 1000 && m_target.getDistance() > 0.0) {
            msg.linear.x = 2 * (m_target.getDistance() / 1000) - 2;
            ROS_INFO("[MOVING BACKWARDS AT %f]", msg.linear.x);
        }

        m_velocity_command_pub.publish(msg);

        // set and publish robot path
        m_robot_path.header.seq = m_seq_robot;
        m_robot_path.header.stamp = ros::Time::now();
        m_robot_path.header.frame_id = "odom";
        m_robot_path_pub.publish(m_robot_path);
        ++m_seq_robot;

        // set and publish target path
        m_target_path.header.seq = m_seq_target;
        m_target_path.header.stamp = ros::Time::now();
        m_target_path.header.frame_id = "odom";
        m_target_path_pub.publish(m_target_path);
        ++m_seq_target;

        // set and publish robot velocity
        std_msgs::Float32 turtlebot_velocity;
        turtlebot_velocity.data = m_turtlebot.getVelocity();
        m_robot_velocity_pub.publish(turtlebot_velocity);

        // set and publish target velocity
        std_msgs::Float32 target_velocity;
        target_velocity.data = m_target.getVelocity();
        m_target_velocity_pub.publish(target_velocity);

        // set old position to calculate velocity
        m_turtlebot.updateOldPose();
        m_target.updateOldPosition();

        // FIXME: throws seg fault if list members get deleted
        // manage list
        if (!m_tracked_persons.empty()) {
            for (auto& p : m_tracked_persons) {
                if (p.getDistance() == 0) {
                    p.setTrackingStatus(false);
                    p.setTarget(false);
                }
            }
        }

        loop_rate.sleep();
    }
}


/**
 * @brief Prints out debugging information including the robot and the tracked persons.
 */
void RobustPeopleFollower::debugPrintout()
{
    system("clear");

    m_turtlebot.printTurtlebotInfo();

    ROS_INFO("target information:");
    m_target.printVerbosePersonInfo();

    ROS_INFO("goal list:");
    for (auto& g : m_goal_list) {
        ROS_INFO("[%f, %f]", g.x, g.y);
    }

    ROS_INFO("list size: %lu", m_tracked_persons.size());
    if (!m_tracked_persons.empty()) {
        for (auto& p : m_tracked_persons) {
            if (p.isTracked()) {
                p.printPersonInfo();
            }
        }
    }
}


/**
 * @brief Sets odometry fields with data from the subscribed odometry topic.
 * @param msg the message from the subscribed topic
 */
void RobustPeopleFollower::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Pose pose;
    pose.position.x = msg->pose.pose.position.x;
    pose.position.y = msg->pose.pose.position.y;
    pose.position.z = msg->pose.pose.position.z;
    pose.orientation.x = msg->pose.pose.orientation.x;
    pose.orientation.y = msg->pose.pose.orientation.y;
    pose.orientation.z = msg->pose.pose.orientation.z;
    pose.orientation.w = msg->pose.pose.orientation.w;
    m_turtlebot.setPose(pose);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.seq = m_seq_robot;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x = msg->pose.pose.position.x;
    pose_stamped.pose.position.y = msg->pose.pose.position.y;
    pose_stamped.pose.position.z = msg->pose.pose.position.z;
    pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
    pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
    pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
    pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;
    m_robot_path.poses.push_back(pose_stamped);
}


/**
 * @brief Manages the list of tracked persons with data received from the skeleton topic.
 * @param msg the message from the subscribed topic
 */
void RobustPeopleFollower::skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg)
{

    // save data from message
    body_tracker_msgs::Skeleton skeleton;

    skeleton.body_id = msg->body_id;
    skeleton.tracking_status = msg->tracking_status;
    skeleton.gesture = msg->gesture;
    skeleton.position2D = msg->position2D;
    skeleton.centerOfMass = msg->centerOfMass;
    skeleton.joint_position_head = msg->joint_position_head;
    skeleton.joint_position_neck = msg->joint_position_neck;
    skeleton.joint_position_shoulder = msg->joint_position_shoulder;
    skeleton.joint_position_spine_top = msg->joint_position_spine_top;
    skeleton.joint_position_spine_mid = msg->joint_position_spine_mid;
    skeleton.joint_position_spine_bottom = msg->joint_position_spine_bottom;
    skeleton.joint_position_left_shoulder = msg->joint_position_left_shoulder;
    skeleton.joint_position_left_elbow = msg->joint_position_left_elbow;
    skeleton.joint_position_left_hand = msg->joint_position_left_hand;
    skeleton.joint_position_right_shoulder = msg->joint_position_right_shoulder;
    skeleton.joint_position_right_elbow = msg->joint_position_right_elbow;
    skeleton.joint_position_right_hand = msg->joint_position_right_hand;

    bool found = false;
    for (auto& p : m_tracked_persons) {
        if (p.getId() == skeleton.body_id) {
            found = true;
            if (p.isTarget()) {
                p.setSkeleton(skeleton);
                p.calculateAbsolutePosition(m_turtlebot.getPose().position.x, m_turtlebot.getPose().position.y,
                                            m_turtlebot.getAngle());
            } else {

                // update information
                p.setSkeleton(skeleton);

                // FIXME: doesn't work as intended
                // check for gestures
                if (skeleton.gesture == 2) {
                    if (p.getGestureBegin() == 0) {
                        p.setGestureBegin(ros::Time::now());
                    }
                } else {
                    if (p.getGestureBegin() != 0) {

                        // TODO: figure out how to play a sound
                        // new target selected after 3 seconds of closing both hands
                        if (ros::Time::now().sec - p.getGestureBegin() >= 3) {
                            p.setTarget(true);
                            m_turtlebot.setStatus(FOLLOWING);

                            // reset gesture beginning time
                            p.setGestureBegin(ros::Time(0));
                        }
                    }
                }
            }
        }
    }

    // save new person if new id has been detected
    if (!found) {
        m_tracked_persons.emplace_back(Person(skeleton));
    }
}
