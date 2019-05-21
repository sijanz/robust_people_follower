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


#include <tf/transform_datatypes.h>

#include "robust_people_follower/robust_people_follower.h"


RobustPeopleFollower::RobustPeopleFollower(const std::string& t_name) : m_robot(Robot{}),
                                                                        m_tracked_persons(std::vector<Person>{}),
                                                                        m_robot_path(nav_msgs::Path{}),
                                                                        m_target_path(nav_msgs::Path{}), m_seq_robot(0),
                                                                        m_seq_target(0)
{
    m_name = t_name;

    m_odom_sub = m_nh.subscribe("/odom", 10, &RobustPeopleFollower::odometryCallback, this);
    m_skeleton_sub = m_nh.subscribe("/body_tracker/skeleton", 10, &RobustPeopleFollower::skeletonCallback, this);

    m_velocity_command_pub = m_nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    m_robot_path_pub = m_nh.advertise<nav_msgs::Path>("robust_people_follower/robot_path", 1000);
    m_target_path_pub = m_nh.advertise<nav_msgs::Path>("robust_people_follower/target_path", 1000);
    m_visualization_pub = m_nh.advertise<visualization_msgs::Marker>("robust_people_follower/markers", 10);
}


RobustPeopleFollower::~RobustPeopleFollower()
{
    ROS_INFO("%s shutting down", m_name.c_str());
    m_odom_sub.shutdown();
    m_skeleton_sub.shutdown();
}


void RobustPeopleFollower::runLoop()
{
    bool in_first_half, in_second_half = false;

    ros::Rate loop_rate(LOOP_FREQUENCY);

    while (ros::ok()) {

        // process callbacks
        ros::spinOnce();

        // set the target's variables
        for (auto& p : m_tracked_persons) {
            if (p.target()) {
                m_target = p;
                break;
            }
        }

        // add new goal to goal list
        m_robot.addNewGoal(m_target);

        // TODO: implement actual searching
        // robot loses target
        if (m_target.distance() == 0 && m_target.distance() == 0 &&
            m_robot.status() == Robot::Status::FOLLOWING) {
            m_robot.status() = Robot::Status::SEARCHING;

            for (auto& p : m_tracked_persons) {
                if (p.target()) {

                    // publish estimation marker
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "odom";
                    marker.header.stamp = ros::Time();
                    marker.ns = "estimation";
                    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.lifetime = ros::Duration(20);
                    marker.pose.position.x = m_target.oldPose().position.x;
                    marker.pose.position.y = m_target.oldPose().position.y;
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

                    // FIXME: use the old vector
                    visualization_msgs::Marker vector;
                    vector.header.frame_id = "odom";
                    vector.header.stamp = ros::Time();
                    vector.ns = "vectors";
                    vector.type = visualization_msgs::Marker::ARROW;
                    vector.action = visualization_msgs::Marker::ADD;
                    vector.pose.position.x = p.oldPose().position.x;
                    vector.pose.position.y = p.oldPose().position.y;
                    vector.pose.position.z = 1.3;

                    tf::Quaternion q = tf::createQuaternionFromYaw(p.angle());
                    vector.pose.orientation.x = q.getX();
                    vector.pose.orientation.y = q.getY();
                    vector.pose.orientation.z = q.getZ();
                    vector.pose.orientation.w = q.getW();

                    vector.scale.x = 0.5 + VECTOR_LENGTH_FACTOR * p.velocity();
                    vector.scale.y = 0.1;
                    vector.scale.z = 0.1;

                    vector.color.a = 1.0;
                    vector.color.r = 1.0;
                    vector.color.g = 0.0;
                    vector.color.b = 0.0;

                    m_visualization_pub.publish(vector);

                }
            }
        }

        // update path for target to be published
        updateTargetPath();

        debugPrintout();

        // TODO: test
        // move the robot
        if (m_robot.status() != Robot::Status::WAITING) {
            geometry_msgs::Twist speed = m_robot.setVelocityCommand(m_target, 1800);

            // DEBUG
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
        m_robot.updateOldPose();
        std::for_each(m_tracked_persons.begin(), m_tracked_persons.end(), [](Person& p) { p.updateOldPose(); });

        // list management
        managePersonList();

        loop_rate.sleep();
    }
}


/**
 * @brief Prints out debugging information including the robot and the tracked persons.
 */
void RobustPeopleFollower::debugPrintout()
{
    system("clear");

    ROS_INFO("ROS time: %d", ros::Time::now().sec);

    m_robot.printInfo();

    ROS_INFO("target information:");
    m_target.printVerboseInfo();

    ROS_INFO("goal list size: %lu", m_robot.goalList().size());

    ROS_INFO("tracked persons: %lu", m_tracked_persons.size());
    if (!m_tracked_persons.empty())
        std::for_each(m_tracked_persons.begin(), m_tracked_persons.end(), [](const Person& p) { p.printInfo(); });
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
    m_robot.pose() = pose;

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

    m_robot.calculateAngle();
    m_robot.calculateVelocity(LOOP_FREQUENCY);
}


// TODO: change logical structure
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
        if (p.id() == skeleton.body_id) {
            found = true;

            // update information
            p.skeleton() = skeleton;
            p.calculateAbsolutePosition(m_robot.pose().position.x, m_robot.pose().position.y,
                                        m_robot.angle());
            p.calculateAngle();
            p.calculateVelocity(LOOP_FREQUENCY);

            // target
            if (p.target()) {

                if (skeleton.centerOfMass.x == 0.0)
                    break;

                // check for gestures
                if (skeleton.gesture == 2 && p.hasCorrectHandHeight()) {
                    if (p.gestureBegin() == ros::Time(0)) {
                        p.gestureBegin() = ros::Time::now();
                    }
                } else {
                    if (p.gestureBegin() != ros::Time(0)) {

                        // TODO: figure out how to play a sound
                        // target chooses to stop being followed
                        if (ros::Time::now().sec - p.gestureBegin().sec >= 3) {
                            p.target() = false;

                            // reset target's information
                            /*
                            m_target.setSkeleton({});
                            m_target.setTarget(false);
                            m_target.setVelocity(0.0);
                            m_target.setAbsolutePosition(geometry_msgs::Point32{});
                            m_target.setGestureBegin(ros::Time(0));
                            m_target.setAngle(0.0);
                            */

                            m_robot.status() = Robot::Status::WAITING;

                            // reset gesture beginning time
                            p.gestureBegin() = ros::Time(0);
                        }
                    }
                }
            }

                // other persons
            else {

                // check for gestures
                if (skeleton.gesture == 2 && p.hasCorrectHandHeight()) {
                    if (p.gestureBegin() == ros::Time(0)) {
                        p.gestureBegin() = ros::Time::now();
                    }
                } else {
                    if (p.gestureBegin() != ros::Time(0)) {

                        // TODO: figure out how to play a sound
                        // new target selected after 3 seconds of closing both hands
                        if (ros::Time::now().sec - p.gestureBegin().sec >= 3) {
                            p.target() = true;
                            m_robot.status() = Robot::Status::FOLLOWING;

                            // reset gesture beginning time
                            p.gestureBegin() = ros::Time(0);
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


void RobustPeopleFollower::publishRobotPath()
{
    m_robot_path.header.seq = m_seq_robot;
    m_robot_path.header.stamp = ros::Time::now();
    m_robot_path.header.frame_id = "odom";
    m_robot_path_pub.publish(m_robot_path);
    ++m_seq_robot;
}


void RobustPeopleFollower::publishTargetPath()
{
    m_target_path.header.seq = m_seq_target;
    m_target_path.header.stamp = ros::Time::now();
    m_target_path.header.frame_id = "odom";
    m_target_path_pub.publish(m_target_path);
    ++m_seq_target;
}


void RobustPeopleFollower::publishPersonMarkers() const
{
    std::vector<visualization_msgs::Marker> person_markers;

    int i = 0;
    for (auto& p : m_tracked_persons) {
        if (p.distance() > 0) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time();
            marker.ns = "persons";
            marker.id = i;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(0.3);
            marker.pose.position.x = p.pose().position.x;
            marker.pose.position.y = p.pose().position.y;
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
            if (p.target()) {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
            }

            person_markers.emplace_back(marker);
            ++i;
        }
    }

    std::for_each(person_markers.begin(), person_markers.end(),
                  [&](const visualization_msgs::Marker& m) { m_visualization_pub.publish(m); });
}


void RobustPeopleFollower::publishPersonVectors() const
{
    std::vector<visualization_msgs::Marker> person_vectors;

    int i = 0;
    for (auto& p : m_tracked_persons) {
        if (p.distance() > 0) {
            visualization_msgs::Marker vector;
            vector.header.frame_id = "odom";
            vector.header.stamp = ros::Time();
            vector.ns = "vectors";
            vector.id = i;
            vector.type = visualization_msgs::Marker::ARROW;
            vector.action = visualization_msgs::Marker::ADD;
            vector.lifetime = ros::Duration(0.3);
            vector.pose.position.x = p.pose().position.x;
            vector.pose.position.y = p.pose().position.y;
            vector.pose.position.z = 1.3;

            tf::Quaternion q = tf::createQuaternionFromYaw(p.angle());
            vector.pose.orientation.x = q.getX();
            vector.pose.orientation.y = q.getY();
            vector.pose.orientation.z = q.getZ();
            vector.pose.orientation.w = q.getW();

            vector.scale.x = 0.5 + VECTOR_LENGTH_FACTOR * p.velocity();
            vector.scale.y = 0.1;
            vector.scale.z = 0.1;

            vector.color.a = 1.0; // Don't forget to set the alpha!
            vector.color.r = 1.0;
            vector.color.g = 0.0;
            vector.color.b = 0.0;

            person_vectors.emplace_back(vector);
            ++i;
        }
    }

    std::for_each(person_vectors.begin(), person_vectors.end(),
                  [&](const visualization_msgs::Marker& m) { m_visualization_pub.publish(m); });
}


void RobustPeopleFollower::publishRobotGoals() const
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

    if (!m_robot.goalList().empty()) {
        for (auto& g : m_robot.goalList()) {
            geometry_msgs::Point p;
            p.x = g.point.x;
            p.y = g.point.y;

            line_strip.points.push_back(p);
            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);
        }
    }

    // publish markers
    m_visualization_pub.publish(line_strip);
    m_visualization_pub.publish(line_list);
}


void RobustPeopleFollower::managePersonList()
{
    if (!m_tracked_persons.empty()) {
        auto it = m_tracked_persons.begin();
        while (it != m_tracked_persons.end()) {
            if (it->distance() == 0 && it->yDeviation() == 0)
                it = m_tracked_persons.erase(it);
            else
                ++it;
        }
    }
}


void RobustPeopleFollower::updateTargetPath()
{
    if (m_target.distance() > 0) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.seq = m_seq_target;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = m_target.pose().position.x;
        pose_stamped.pose.position.y = m_target.pose().position.y;
        m_target_path.poses.push_back(pose_stamped);
    }
}
