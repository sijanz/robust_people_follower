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
#include <memory>

#include "robust_people_follower/robust_people_follower.h"


RobustPeopleFollower::RobustPeopleFollower(const std::string& t_name)
        : m_robot{}, m_robot_path{new nav_msgs::Path{}}, m_target_path{new nav_msgs::Path{}}, m_seq_robot{},
          m_seq_target{}
{
    m_name = t_name;

    m_odom_sub = m_nh.subscribe("/odom", 10, &RobustPeopleFollower::odometryCallback, this);
    m_skeleton_sub = m_nh.subscribe("/body_tracker/skeleton", 10, &RobustPeopleFollower::skeletonCallback, this);

    m_velocity_command_pub = m_nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    m_robot_path_pub = m_nh.advertise<nav_msgs::Path>("robust_people_follower/robot_path", 10);
    m_target_path_pub = m_nh.advertise<nav_msgs::Path>("robust_people_follower/target_path", 10);
    m_visualization_pub = m_nh.advertise<visualization_msgs::Marker>("robust_people_follower/markers", 10);
}


RobustPeopleFollower::~RobustPeopleFollower()
{
    ROS_INFO_STREAM(m_name << " shutting down");
    m_odom_sub.shutdown();
    m_skeleton_sub.shutdown();
}


void RobustPeopleFollower::runLoop()
{
    ros::Rate loop_rate{LOOP_FREQUENCY};

    auto search_time{ros::Time{0}};

    while (ros::ok()) {

        // create marker to be published if the target is lost
        auto last_point_marker{lastPointMarker()};

        // set the point where the target is last seen
        geometry_msgs::Point32 last_target_point{};
        last_target_point.x = m_robot.target().oldPose().position.x;
        last_target_point.y = m_robot.target().oldPose().position.y;

        // process callbacks
        ros::spinOnce();

        // set the target's variables
        for (auto& p : *m_robot.trackedPersons()) {
            if (p.target()) {
                m_robot.target() = p;
                break;
            }
        }

        // print out program information on the screen
        debugPrintout();

        // set status to "LOS_LOST" if target is lost
        if (m_robot.target().distance() == 0 && m_robot.status() == Robot::Status::FOLLOWING)
            m_robot.status() = Robot::Status::LOS_LOST;

        // estimate the target's position if the line of sight to the target is lost
        if (m_robot.status() == Robot::Status::LOS_LOST || m_robot.status() == Robot::Status::SEARCHING) {
            m_robot.estimateTargetPosition(last_target_point.x, last_target_point.y);
            m_visualization_pub.publish(last_point_marker);
            m_visualization_pub.publish(targetEstimationMarker());
        }

        // reidentify after 2 seconds
        if (m_robot.status() == Robot::Status::SEARCHING) {
            if (search_time == ros::Time{0})
                search_time = ros::Time::now() + ros::Duration{2};
            else {
                if (ros::Time::now() > search_time)
                    m_robot.reIdentify();
            }
        }

        // FIXME: not working correctly
        // re-identify the target if the robot is at the target's last known position
//        if (m_robot.status() == Robot::Status::SEARCHING)
//            m_robot.reIdentify();

        // add new goal to goal list
        if (m_robot.target().distance() > FOLLOW_THRESHOLD)
            m_robot.addNewWaypoint(4);

        // update path for target to be published
        updateTargetPath();

        // move the robot
        if (m_robot.status() != Robot::Status::WAITING)
            m_velocity_command_pub.publish(m_robot.velocityCommand(FOLLOW_THRESHOLD));

        // publish markers to view in RViz
        publishPaths();
        publishPersonMarkers();
        publishWaypoints();

        // set old positions to calculate velocities
        m_robot.updatePose();
        for (auto& p : *m_robot.trackedPersons())
            p.updatePose();

        // delete persons that are no longer in line of sight
        m_robot.managePersonList();

        loop_rate.sleep();
    }
}


/**
 * @brief Prints out debugging information including the robot and the tracked persons.
 */
void RobustPeopleFollower::debugPrintout()
{
    system("clear");

    ROS_INFO_STREAM("ROS time: " << ros::Time::now().sec);
    m_robot.printInfo();

    // TODO: to Robot
    ROS_INFO_STREAM("target information:");
    m_robot.target().printVerboseInfo();

    ROS_INFO_STREAM("goal list size: " << m_robot.waypoints()->size());

    ROS_INFO_STREAM("tracked persons: " << m_robot.trackedPersons()->size());
    for (const auto& p : *m_robot.trackedPersons())
        p.printInfo();
}


/**
 * @brief Sets odometry fields with data from the subscribed odometry topic.
 * @param msg the message from the subscribed topic
 */
void RobustPeopleFollower::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Pose pose{};
    pose.position.x = msg->pose.pose.position.x;
    pose.position.y = msg->pose.pose.position.y;
    pose.position.z = msg->pose.pose.position.z;
    pose.orientation.x = msg->pose.pose.orientation.x;
    pose.orientation.y = msg->pose.pose.orientation.y;
    pose.orientation.z = msg->pose.pose.orientation.z;
    pose.orientation.w = msg->pose.pose.orientation.w;
    m_robot.pose() = pose;

    geometry_msgs::PoseStamped pose_stamped{};
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
    m_robot_path->poses.push_back(pose_stamped);

    m_robot.calculateAngle();
    m_robot.calculateVelocity(LOOP_FREQUENCY);
}


/**
 * @brief Manages the list of tracked persons with data received from the skeleton topic.
 * @param msg the message from the subscribed topic
 */
void RobustPeopleFollower::skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg)
{

    // save data from message
    body_tracker_msgs::Skeleton skeleton{};

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


    auto id{skeleton.body_id};
    auto p{std::find(m_robot.trackedPersons()->begin(), m_robot.trackedPersons()->end(), id)};

    // person is already in the list
    if (p != m_robot.trackedPersons()->end()) {

        // update information
        p->skeleton() = skeleton;
        p->calculateAbsolutePosition(m_robot.pose().position.x, m_robot.pose().position.y, m_robot.angle());
        p->calculateAngle();
        p->calculateVelocity(LOOP_FREQUENCY);

        // target
        if (p->target()) {

            // check for gestures
            if (skeleton.gesture == 2 && p->correctHandHeight()) {
                if (p->gestureBegin() == ros::Time{0}) {
                    p->gestureBegin() = ros::Time::now();
                }
            } else {
                if (p->gestureBegin() != ros::Time{0}) {

                    // TODO: figure out how to play a sound
                    // target chooses to stop being followed
                    if (ros::Time::now().sec - p->gestureBegin().sec >= 3) {
                        p->target() = false;
                        m_robot.target() = Person{};
                        m_robot.status() = Robot::Status::WAITING;
                        m_robot.waypoints()->clear();
                        p->gestureBegin() = ros::Time{0};
                    }
                }
            }
        }

            // other persons
        else {

            // check for gestures
            if (skeleton.gesture == 2 && p->correctHandHeight()) {
                if (p->gestureBegin() == ros::Time{0})
                    p->gestureBegin() = ros::Time::now();
            } else {
                if (p->gestureBegin() != ros::Time{0}) {

                    // TODO: figure out how to play a sound
                    // new target selected after 3 seconds of closing both hands
                    if (ros::Time::now().sec - p->gestureBegin().sec >= 3) {
                        p->target() = true;
                        m_robot.status() = Robot::Status::FOLLOWING;

                        // reset gesture beginning time
                        p->gestureBegin() = ros::Time{0};
                    }
                }
            }
        }
    }

        // add new person if skeleton id is not in list
    else
        m_robot.trackedPersons()->emplace_back(Person{skeleton});
}


void RobustPeopleFollower::publishPaths()
{
    m_robot_path->header.seq = m_seq_robot;
    m_robot_path->header.stamp = ros::Time::now();
    m_robot_path->header.frame_id = "odom";
    m_robot_path_pub.publish(*m_robot_path);
    ++m_seq_robot;

    m_target_path->header.seq = m_seq_target;
    m_target_path->header.stamp = ros::Time::now();
    m_target_path->header.frame_id = "odom";
    m_target_path_pub.publish(*m_target_path);
    ++m_seq_target;
}


void RobustPeopleFollower::publishPersonMarkers() const
{
    std::vector<visualization_msgs::Marker> person_markers{}, person_vectors{};

    auto i{0};
    for (const auto& p : *m_robot.trackedPersons()) {
        if (p.distance() > 0) {
            visualization_msgs::Marker marker{};
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "persons";
            marker.id = i;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration{0.3};
            marker.pose.position.x = p.pose().position.x;
            marker.pose.position.y = p.pose().position.y;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.5;

            marker.mesh_resource = "package://robust_people_follower/meshes/standing.dae";

            // show a green marker for a tracked person
            if (p.target()) {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
            }

            person_markers.emplace_back(marker);

            visualization_msgs::Marker vector{};
            vector.header.frame_id = "odom";
            vector.header.stamp = ros::Time::now();
            vector.ns = "vectors";
            vector.id = i;
            vector.type = visualization_msgs::Marker::ARROW;
            vector.action = visualization_msgs::Marker::ADD;
            vector.lifetime = ros::Duration{0.3};
            vector.pose.position.x = p.pose().position.x;
            vector.pose.position.y = p.pose().position.y;
            vector.pose.position.z = 1.3;

            tf::Quaternion q = tf::createQuaternionFromYaw(p.angle());
            vector.pose.orientation.x = q.getX();
            vector.pose.orientation.y = q.getY();
            vector.pose.orientation.z = q.getZ();
            vector.pose.orientation.w = q.getW();

            vector.scale.x = 0.0 + VECTOR_LENGTH_FACTOR * p.velocity();
            vector.scale.y = 0.1;
            vector.scale.z = 0.1;

            vector.color.a = 1.0;
            vector.color.r = 1.0;

            person_vectors.emplace_back(vector);

            ++i;
        }
    }

    for (const auto& m : person_markers)
        m_visualization_pub.publish(m);
    for (const auto& m : person_vectors)
        m_visualization_pub.publish(m);
}


void RobustPeopleFollower::publishWaypoints() const
{
    visualization_msgs::Marker line_strip{}, line_list{};
    line_strip.header.frame_id = line_list.header.frame_id = "odom";
    line_strip.ns = line_list.ns = "waypoints";
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

    for (const auto& w : *m_robot.waypoints()) {
        geometry_msgs::Point p{};
        p.x = w.point.x;
        p.y = w.point.y;

        line_strip.points.push_back(p);
        line_list.points.push_back(p);
        p.z += 1.0;
        line_list.points.push_back(p);
    }

    // publish markers
    m_visualization_pub.publish(line_strip);
    m_visualization_pub.publish(line_list);
}


void RobustPeopleFollower::updateTargetPath()
{
    if (m_robot.target().distance() > 0) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.seq = m_seq_target;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = m_robot.target().pose().position.x;
        pose_stamped.pose.position.y = m_robot.target().pose().position.y;
        m_target_path->poses.push_back(pose_stamped);
    }
}


visualization_msgs::Marker RobustPeopleFollower::lastPointMarker() const
{
    visualization_msgs::Marker marker{};
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "last_seen";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration{0.3};
    marker.pose.position.x = m_robot.target().oldPose().position.x;
    marker.pose.position.y = m_robot.target().oldPose().position.y;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    marker.mesh_resource = "package://robust_people_follower/meshes/standing.dae";

    return marker;
}


visualization_msgs::Marker RobustPeopleFollower::targetEstimationMarker() const
{
    visualization_msgs::Marker marker{};
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "estimation";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration{0.3};
    marker.pose.position.x = m_robot.estimatedTargetPosition().x;
    marker.pose.position.y = m_robot.estimatedTargetPosition().y;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.5;
    marker.color.b = 1.0;
    marker.mesh_resource = "package://robust_people_follower/meshes/standing.dae";

    return marker;
}
