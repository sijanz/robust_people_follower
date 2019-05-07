#include <stdlib.h>

#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "turtlebot.h"


// ####### GLOBAL VARIABLES #######

// instance of the turtlebot robot
Turtlebot turtlebot;

// list of persons in the frame
std::vector<Person> tracked_persons;

// stores the path of the robot
nav_msgs::Path robot_path;

// sequence number for path
uint32_t seq = 0;


// TODO: replace with path following algorithm
// distance and deviation of target
float target_distance = 0;
float target_y_deviation = 0;


// ####### FUNCTION PROTOTYPES #######

void debugPrintout();

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr &msg);


// ####### ENTRY POINT #######

int main(int argc, char **argv)
{
    ros::init(argc, argv, "body_tracker");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odometryCallback);
    ros::Subscriber skeleton_sub = nh.subscribe("/body_tracker/skeleton", 10, skeletonCallback);

    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        debugPrintout();

        for (std::vector<Person>::iterator iter = tracked_persons.begin(); iter != tracked_persons.end(); ++iter) {
            if (iter->isTarget()) {
                target_distance = iter->getDistance();
                target_y_deviation = iter->getYDeviation();
            }
        }


        // move the robot
        // message to store velocity commands in
        geometry_msgs::Twist msg;

        // TODO: make smoother
        if (target_y_deviation < -100) {
            msg.angular.z = -0.5;
            ROS_INFO("[TURNING LEFT at %f]", msg.angular.z);
        } else if (target_y_deviation > 100) {
            msg.angular.z = 0.5;
            ROS_INFO("[TURNING RIGHT at %f]", msg.angular.z);
        } else
            msg.angular.z = 0;

        // publish velocity command for rotation
        velocity_pub.publish(msg);

        if (target_distance > 1800) {
            msg.linear.x = 0.2;
            ROS_INFO("[MOVING FORWARDS at %f]", msg.linear.x);
        } else if (target_distance < 1000 && target_distance > 0) {
            msg.linear.x = -0.2;
            ROS_INFO("[MOVING BACKWARDS at %f]", msg.linear.x);
        }

        // publish velocity command for moving straight
        velocity_pub.publish(msg);

        // FIXME: throws seg fault if list members get deleted
        if (!tracked_persons.empty()) {
            for (std::vector<Person>::iterator iter = tracked_persons.begin(); iter != tracked_persons.end(); ++iter) {
                if (iter->getDistance() == 0) {
                    iter->setTrackingStatus(false);
                    iter->setTarget(false);
                }
            }
        }


        robot_path.header.seq = seq;
        robot_path.header.stamp = ros::Time::now();
        robot_path.header.frame_id = "odom";

        path_pub.publish(robot_path);
        ++seq;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


/**
 * @brief Prints out debugging information including the robot and the tracked persons.
 */
void debugPrintout()
{
    system("clear");

    turtlebot.printTurtlebotInfo();

    ROS_INFO("target information:");
    ROS_INFO("  distance: %f", target_distance);
    ROS_INFO("  y-deviation: %f\n", target_y_deviation);

    ROS_INFO("list size: %lu\n", tracked_persons.size());

    if (!tracked_persons.empty()) {
        for (std::vector<Person>::iterator iter = tracked_persons.begin(); iter != tracked_persons.end(); ++iter) {
            if (iter->isTracked()) {
                iter->printPersonInfo();
            }
        }
    }
}


/**
 * @brief Sets odometry fields with data from the subscribed odometry topic.
 * @param msg the message from the subscribed topic
 */
void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    turtlebot.setPose(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

    geometry_msgs::PoseStamped pose;
    pose.header.seq = seq;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "odom";

    pose.pose.position.x = msg->pose.pose.position.x;
    pose.pose.position.y = msg->pose.pose.position.y;
    pose.pose.position.z = msg->pose.pose.position.z;

    pose.pose.orientation.z = msg->pose.pose.orientation.z;
    pose.pose.orientation.w = msg->pose.pose.orientation.w;

    robot_path.poses.push_back(pose);
}


/**
 * @brief Manages the list of tracked persons with data received from the skeleton topic.
 * @param msg the message from the subscribed topic
 */
void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr &msg)
{

    // save data from message
    body_tracker_msgs::Skeleton skeleton;

    skeleton.body_id = msg->body_id;
    skeleton.tracking_status = msg->tracking_status;
    skeleton.gesture = msg->gesture;
    skeleton.position2D = msg->position2D;
    skeleton.centerOfMass = msg->centerOfMass;
    skeleton.joint_position_head = msg->joint_position_head;
    skeleton.joint_position_neck =  msg->joint_position_neck;
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
    for (std::vector<Person>::iterator iter = tracked_persons.begin(); iter != tracked_persons.end(); ++iter) {
        if (iter->getId() == skeleton.body_id) {
            found = true;
            if (iter->isTarget()) {
                iter->setSkeleton(skeleton);
            } else {

                // update information
                iter->setSkeleton(skeleton);

                // FIXME
                // check for gestures
                if (skeleton.gesture == 2) {
                    if (iter->getGestureBegin() == 0) {
                        iter->setGestureBegin(ros::Time::now());
                    }
                } else {
                    if (iter->getGestureBegin() != 0) {

                        // TODO: figure out how to play a sound
                        // new target selected after 3 seconds of closing both hands
                        if (ros::Time::now().sec - iter->getGestureBegin() >= 3) {
                            iter->setTarget(true);
                            turtlebot.setStatus(FOLLOWING);

                            // reset gesture beginning time
                            iter->setGestureBegin(ros::Time(0));
                        }
                    }
                }
            }
        }
    }

    // save new person if new id has been detected
    if (!found) {
        tracked_persons.emplace_back(Person(skeleton));
    }
}
