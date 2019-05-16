/*********************************************************************
* Software License Agreement (BSD 3-Clause License)
*
*  Copyright (c) 2019, Simon Janzon
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <stdlib.h>
#include <cmath>
#include <deque>

#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include "turtlebot.h"


class RobustPeopleFollowerNode
{
public:

    // TODO: make members private
    ros::NodeHandle nh;

    // subscribers
    ros::Subscriber odom_sub;
    ros::Subscriber skeleton_sub;

    explicit RobustPeopleFollowerNode(const std::string& t_name);
    ~RobustPeopleFollowerNode();
    void runLoop();
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg);

private:

    // name of the node
    std::string m_name;

    // frequency of the main loop
    const double LOOP_FREQUENCY = 10.0;

    // factor for person vector length
    const double VECTOR_LENGTH_FACTOR = 1.0;

    // publishers
    ros::Publisher m_velocity_command_pub;
    ros::Publisher m_robot_path_pub;
    ros::Publisher m_target_path_pub;
    ros::Publisher m_visualization_pub;

    // instance of the turtlebot robot
    Turtlebot m_turtlebot;

    // instance of the target to follow
    Person m_target;

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

    // list that holds goals, at maximum 10
    std::deque<geometry_msgs::PointStamped> m_goal_list;

    // is used if a goal has been set for the current second
    int m_last_goal_time;

    // helper methods to keep the main loop tidy
    void debugPrintout();
    void setTarget();
    void publishRobotPath();
    void publishTargetPath();
    void publishPersonMarkers() const;
    void publishPersonVectors() const;
    void publishRobotGoals() const;
    void managePersonList();
    void manageGoalList();
    void addNewGoal();
    void updateTargetPath();
};


// ####### ENTRY POINT #######

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robust_people_follower");
    RobustPeopleFollowerNode node(ros::this_node::getName());
    node.odom_sub = node.nh.subscribe("/odom", 10, &RobustPeopleFollowerNode::odometryCallback, &node);
    node.skeleton_sub = node.nh.subscribe("/body_tracker/skeleton", 10, &RobustPeopleFollowerNode::skeletonCallback,
                                          &node);
    node.runLoop();

    return 0;
}


// ####### METHOD DEFINITIONS #######

RobustPeopleFollowerNode::RobustPeopleFollowerNode(const std::string& t_name)
{
    m_name = t_name;
    m_velocity_command_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    m_robot_path_pub = nh.advertise<nav_msgs::Path>("robust_people_follower/robot_path", 1000);
    m_target_path_pub = nh.advertise<nav_msgs::Path>("robust_people_follower/target_path", 1000);
    m_visualization_pub = nh.advertise<visualization_msgs::Marker>("robust_people_follower/markers", 10);
    m_turtlebot = {};
    m_tracked_persons = {};
    m_robot_path = m_target_path = {};
    m_seq_robot = m_seq_target = {};
    m_goal_list = {};
    m_last_goal_time = 0;
}


RobustPeopleFollowerNode::~RobustPeopleFollowerNode()
{
    ROS_INFO("%s shutting down", m_name.c_str());
    odom_sub.shutdown();
    skeleton_sub.shutdown();
}


void RobustPeopleFollowerNode::runLoop()
{
    ros::Rate loop_rate(LOOP_FREQUENCY);

    while (ros::ok()) {

        // process callbacks
        ros::spinOnce();

        // set the target's variables
        setTarget();

        // add new goal to goal list
        addNewGoal();

        // TODO: implement actual searching
        // robot loses target
        if (m_target.getDistance() == 0 && m_target.getYDeviation() == 0 &&
            m_turtlebot.getStatus() == Turtlebot::Status::FOLLOWING) {
            m_turtlebot.setStatus(Turtlebot::Status::SEARCHING);

            for (auto& p : m_tracked_persons) {
                if (p.isTarget()) {

                    // publish estimation marker
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "odom";
                    marker.header.stamp = ros::Time();
                    marker.ns = "estimation";
                    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = m_target.getOldAbsolutePosition().x;
                    marker.pose.position.y = m_target.getOldAbsolutePosition().y;
                    marker.pose.position.z = 0.0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 1.0;
                    marker.scale.y = 1.0;
                    marker.scale.z = 1.0;

                    marker.color.a = 0.5;
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;

                    marker.mesh_resource = "package://robust_people_follower/meshes/standing.dae";

                    m_visualization_pub.publish(marker);
                }
            }
        }

        // update path for target to be published
        updateTargetPath();

        debugPrintout();

        // TODO: test
        // move the robot
        if (m_target.getDistance() != 0) {
            geometry_msgs::Twist speed;
            speed = m_turtlebot.setVelocityCommand(m_target, m_goal_list, speed);
            ROS_INFO("velocity message: linear: %f, angular: %f", speed.linear.x, speed.angular.z);
            m_velocity_command_pub.publish(speed);
        }


        // publish markers to view in RViz
        publishRobotPath();
        publishTargetPath();
        publishPersonMarkers();
        publishPersonVectors();
        publishRobotGoals();

        // set old position to calculate velocity
        m_turtlebot.updateOldPose();
        m_target.updateOldPosition();
        for (auto& p : m_tracked_persons) {
            p.updateOldPosition();
        }

        // list management
        managePersonList();

        // TODO: test
        manageGoalList();

        loop_rate.sleep();
    }
}


/**
 * @brief Prints out debugging information including the robot and the tracked persons.
 */
void RobustPeopleFollowerNode::debugPrintout()
{
    system("clear");
    ROS_INFO("ROS time: %d", ros::Time::now().sec);

    m_turtlebot.printTurtlebotInfo();

    ROS_INFO("target information:");
    m_target.printVerbosePersonInfo();

    ROS_INFO("goal list:");
    for (auto& g : m_goal_list) {
        ROS_INFO("[%f, %f] %d", g.point.x, g.point.y, g.header.stamp.sec);
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
void RobustPeopleFollowerNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
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

    m_turtlebot.calculateVelocity(LOOP_FREQUENCY);
}


// TODO: change logical structure
/**
 * @brief Manages the list of tracked persons with data received from the skeleton topic.
 * @param msg the message from the subscribed topic
 */
void RobustPeopleFollowerNode::skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg)
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
        if (p.getId() == skeleton.body_id && p.getDistance() > 0) {
            found = true;

            // update information
            p.setSkeleton(skeleton);
            p.calculateAbsolutePosition(m_turtlebot.getPose().position.x, m_turtlebot.getPose().position.y,
                                        m_turtlebot.getAngle());
            p.calculateVelocity(LOOP_FREQUENCY);

            // target
            if (p.isTarget()) {

                // check for gestures
                if (skeleton.gesture == 2 && p.hasCorrectHandHeight()) {
                    if (p.getGestureBegin() == 0) {
                        p.setGestureBegin(ros::Time::now());
                    }
                } else {
                    if (p.getGestureBegin() != 0) {

                        // TODO: figure out how to play a sound
                        // target chooses to stop being followed
                        if (ros::Time::now().sec - p.getGestureBegin() >= 3) {
                            p.setTarget(false);

                            // reset target's information
                            m_target.setSkeleton({});
                            m_target.setTarget(false);
                            m_target.setVelocity(0.0);
                            m_target.setAbsolutePosition(geometry_msgs::Point32{});
                            m_target.setGestureBegin(ros::Time(0));
                            m_target.setAngle(0.0);

                            m_turtlebot.setStatus(Turtlebot::Status::WAITING);

                            // reset gesture beginning time
                            p.setGestureBegin(ros::Time(0));
                        }
                    }
                }
            }

                // other persons
            else {

                // check for gestures
                if (skeleton.gesture == 2 && p.hasCorrectHandHeight()) {
                    if (p.getGestureBegin() == 0) {
                        p.setGestureBegin(ros::Time::now());
                    }
                } else {
                    if (p.getGestureBegin() != 0) {

                        // TODO: figure out how to play a sound
                        // new target selected after 3 seconds of closing both hands
                        if (ros::Time::now().sec - p.getGestureBegin() >= 3) {
                            p.setTarget(true);
                            m_turtlebot.setStatus(Turtlebot::Status::FOLLOWING);

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


void RobustPeopleFollowerNode::setTarget()
{
    for (auto& p : m_tracked_persons) {
        if (p.isTarget()) {
            m_target.setSkeleton(p.getSkeleton());
            m_target.setAbsolutePosition(p.getAbsolutePosition());
            m_target.calculateVelocity(LOOP_FREQUENCY);
            m_target.setAngle(p.getAngle());
            break;
        }
    }
}


void RobustPeopleFollowerNode::publishRobotPath()
{
    m_robot_path.header.seq = m_seq_robot;
    m_robot_path.header.stamp = ros::Time::now();
    m_robot_path.header.frame_id = "odom";
    m_robot_path_pub.publish(m_robot_path);
    ++m_seq_robot;
}


void RobustPeopleFollowerNode::publishTargetPath()
{
    m_target_path.header.seq = m_seq_target;
    m_target_path.header.stamp = ros::Time::now();
    m_target_path.header.frame_id = "odom";
    m_target_path_pub.publish(m_target_path);
    ++m_seq_target;
}


// FIXME: delete old markers at (0,0)
void RobustPeopleFollowerNode::publishPersonMarkers() const
{
    std::vector<visualization_msgs::Marker> person_markers;

    int i = 0;
    for (auto& p : m_tracked_persons) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "persons";
        marker.id = i;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = p.getAbsolutePosition().x;
        marker.pose.position.y = p.getAbsolutePosition().y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;

        marker.mesh_resource = "package://robust_people_follower/meshes/standing.dae";

        // show a green marker for a tracked person
        if (p.isTarget()) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
        }

        person_markers.emplace_back(marker);
        ++i;
    }

    for (auto& m : person_markers) {
        m_visualization_pub.publish(m);
    }
}


void RobustPeopleFollowerNode::publishPersonVectors() const
{
    std::vector<visualization_msgs::Marker> person_vectors;

    int i = 0;
    for (auto& p : m_tracked_persons) {
        visualization_msgs::Marker vector;
        vector.header.frame_id = "odom";
        vector.header.stamp = ros::Time();
        vector.ns = "vectors";
        vector.id = i;
        vector.type = visualization_msgs::Marker::ARROW;
        vector.action = visualization_msgs::Marker::ADD;
        vector.pose.position.x = p.getAbsolutePosition().x;
        vector.pose.position.y = p.getAbsolutePosition().y;
        vector.pose.position.z = 1.3;

        tf::Quaternion q = tf::createQuaternionFromYaw(p.getAngle());
        vector.pose.orientation.x = q.getX();
        vector.pose.orientation.y = q.getY();
        vector.pose.orientation.z = q.getZ();
        vector.pose.orientation.w = q.getW();

        vector.scale.x = 0.5 + VECTOR_LENGTH_FACTOR * p.getVelocity();
        vector.scale.y = 0.1;
        vector.scale.z = 0.1;

        vector.color.a = 1.0; // Don't forget to set the alpha!
        vector.color.r = 1.0;
        vector.color.g = 0.0;
        vector.color.b = 0.0;

        person_vectors.emplace_back(vector);
        ++i;
    }

    for (auto& v : person_vectors) {
        m_visualization_pub.publish(v);
    }
}


void RobustPeopleFollowerNode::publishRobotGoals() const
{
    visualization_msgs::Marker line_strip, line_list;
    line_strip.header.frame_id = line_list.header.frame_id = "odom";
    line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    line_strip.id = 1;
    line_list.id = 2;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for (auto& g : m_goal_list) {
        geometry_msgs::Point p;
        p.x = g.point.x;
        p.y = g.point.y;

        line_strip.points.push_back(p);
        line_list.points.push_back(p);
        p.z += 1.0;
        line_list.points.push_back(p);
    }

    // publish markers
    m_visualization_pub.publish(line_strip);
    m_visualization_pub.publish(line_list);
}


void RobustPeopleFollowerNode::managePersonList()
{
    if (!m_tracked_persons.empty()) {
        auto it = m_tracked_persons.begin();
        while (it != m_tracked_persons.end()) {
            if (it->getDistance() == 0 && it->getYDeviation() == 0) // TODO: && !it->isTarget()
                it = m_tracked_persons.erase(it);
            else
                ++it;
        }
    }
}


// TODO: test
void RobustPeopleFollowerNode::manageGoalList()
{
    if (!m_goal_list.empty()) {
        auto& g = m_goal_list[0];
        if (ros::Time::now().sec - g.header.stamp.sec > 5)
            m_goal_list.pop_front();
    }
}


void RobustPeopleFollowerNode::addNewGoal()
{

    // only if target is above threshold distance
    if (m_target.getDistance() > 1800 && m_last_goal_time != ros::Time::now().sec) {
        geometry_msgs::PointStamped position;
        position.header.stamp = ros::Time::now();
        position.point.x = m_target.getAbsolutePosition().x;
        position.point.y = m_target.getAbsolutePosition().y;
        position.point.z = 0.0;
        m_goal_list.emplace_back(position);
        m_last_goal_time = ros::Time::now().sec;
    }
    if (m_goal_list.size() > 10)
        m_goal_list.pop_front();
}


void RobustPeopleFollowerNode::updateTargetPath()
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.seq = m_seq_target;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x = m_target.getAbsolutePosition().x;
    pose_stamped.pose.position.y = m_target.getAbsolutePosition().y;
    pose_stamped.pose.position.z = m_target.getAbsolutePosition().z;
    m_target_path.poses.push_back(pose_stamped);
}
