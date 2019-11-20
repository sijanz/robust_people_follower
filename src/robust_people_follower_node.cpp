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
#include <fstream>
#include <chrono>
#include <ctime>

#include "robust_people_follower/robust_people_follower_node.h"


RobustPeopleFollower::RobustPeopleFollower(const std::string& t_name)
        : m_emergency_stop{false}, m_status_module{}, m_tracking_module{}, m_control_module{}, m_recovery_module{}
{
    m_name = t_name;

    m_bumper_sub = m_nh.subscribe("/mobile_base/events/bumper", 10, &RobustPeopleFollower::bumperCallback, this);
    m_odom_sub = m_nh.subscribe("/odom", 10, &RobustPeopleFollower::odometryCallback, this);
    m_skeleton_sub = m_nh.subscribe("/body_tracker/skeleton", 10, &RobustPeopleFollower::skeletonCallback, this);

    m_velocity_command_pub = m_nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    m_visualization_pub = m_nh.advertise<visualization_msgs::Marker>("robust_people_follower/markers", 10);
}


RobustPeopleFollower::~RobustPeopleFollower()
{
    ROS_INFO_STREAM(m_name << " shutting down");

    m_bumper_sub.shutdown();
    m_odom_sub.shutdown();
    m_skeleton_sub.shutdown();
    m_velocity_command_pub.shutdown();
    m_visualization_pub.shutdown();
}


void RobustPeopleFollower::runLoop()
{
    auto loop_rate{ros::Rate{LOOP_FREQUENCY}};

    // LOG
//    std::ofstream file{};
//    file.open("/home/simon/log.csv");
//    file << "t,x,y,d,y_d,v,m_v,m_m_v,th,m_th,m_m_th,p_x,p_y,p_v,d_t\n";
//    auto start_time{ros::Time::now().toSec()};

    while (ros::ok()) {

        auto start{std::chrono::system_clock::now()};

        processCallbacks();

        // check the robot's status
        switch (m_status_module.status()) {
            case StatusModule::Status::FOLLOWING:
                followTarget();
                break;
            case StatusModule::Status::LOS_LOST:
                searchForTarget();
                break;
            case StatusModule::Status::SEARCHING:
                searchForTarget();
                break;
            default:
                break;
        }

        // publish markers to view in RViz
        publishPersonMarkers();
        publishWaypoints();

        auto end{std::chrono::system_clock::now()};

        std::chrono::duration<double> elapsed_time{end - start};

        // print out program information on the screen
        debugPrintout();

        // LOG
//        auto current_time{ros::Time::now().toSec() - start_time};
//        if (m_status_module.status() == StatusModule::Status::FOLLOWING) {
//            file << current_time << "," << m_tracking_module.target().pose().pose.position.x << ","
//                 << m_tracking_module.target().pose().pose.position.y << ","
//                 << m_tracking_module.target().distance() / 1000 << "," << m_tracking_module.target().yDeviation()
//                 << "," << m_tracking_module.target().velocity() << "," << m_tracking_module.target().meanVelocity()
//                 << "," << m_tracking_module.target().angle()
//                 << "," << m_tracking_module.target().meanAngle() << ",0,0,0," << elapsed_time.count() << "\n";
//        } else if (m_status_module.status() == StatusModule::Status::LOS_LOST
//                   || m_status_module.status() == StatusModule::Status::SEARCHING) {
//            file << current_time << ",0,0,0,0,0,0,0,0," << m_recovery_module.predictedTargetPosition().x << ","
//                 << m_recovery_module.predictedTargetPosition().y << "," << m_recovery_module.predictedVelocity() << ","
//                 << elapsed_time.count() << "\n";
//        }

        // delete entries of persons that aren't tracked anymore
        m_tracking_module.managePersonList();

        loop_rate.sleep();
    }
}


void RobustPeopleFollower::processCallbacks()
{
    ros::spinOnce();

    // do an emergency stop if the flag is set
    if (m_emergency_stop) {
        m_status_module.status() = StatusModule::Status::WAITING;

        // publish Twist message with values initialized to 0
        m_velocity_command_pub.publish(geometry_msgs::Twist{});

        // reset target information
        m_tracking_module.target() = Person{};

        m_emergency_stop = false;
    }

    // check if a target is lost
    m_tracking_module.checkForTargetLoss(m_status_module.status());

    // reset flags to only process data once in a loop iteration
    m_status_module.valuesSet() = false;
    m_tracking_module.target().valuesSet() = false;
    for (auto& p : *m_tracking_module.trackedPersons())
        p.valuesSet() = false;
}


void RobustPeopleFollower::debugPrintout() const
{
//    system("clear");
    ROS_INFO_STREAM("\n");

    ROS_INFO_STREAM("ROS time: " << ros::Time::now().sec);
    m_status_module.printInfo();

    if (m_status_module.status() != StatusModule::Status::WAITING) {
        ROS_INFO_STREAM("target information:");
        m_tracking_module.target().printVerboseInfo();

        ROS_INFO_STREAM("goal list size: " << m_control_module.waypoints()->size());
    }

    ROS_INFO_STREAM("tracked persons: " << m_tracking_module.trackedPersons()->size());
    for (const auto& p : *m_tracking_module.trackedPersons())
        p.printInfo();
}

void RobustPeopleFollower::followTarget()
{
    m_velocity_command_pub.publish(m_control_module.velocityCommand(m_status_module.status(),
                                                                    m_status_module.pose(),
                                                                    m_tracking_module.target(), FOLLOW_THRESHOLD));
}


void RobustPeopleFollower::searchForTarget()
{
    if (m_tracking_module.target().lastSeen() < SEARCH_TIMEOUT) {

        // predict the target's position according to the CTRA model
        m_recovery_module.predictTargetPosition(m_tracking_module.target(), 1.0);

        // look at the predicted position if the last waypoint is reached
        if (m_control_module.waypoints()->empty()) {
            m_status_module.status() = StatusModule::Status::SEARCHING;
            m_velocity_command_pub.publish(ControlModule::velocityCommand(m_status_module.pose(),
                                                                          m_recovery_module.predictedTargetPosition()));

            // replicate the target's path
        } else
            m_velocity_command_pub.publish(m_control_module.velocityCommand(m_status_module.status(),
                                                                            m_status_module.pose(),
                                                                            m_tracking_module.target(),
                                                                            FOLLOW_THRESHOLD));


        // TODO: move to recovery module
        auto theta{Object2DSpace::yawFromPose(m_status_module.pose())};

        auto rotation{tf::Matrix3x3{
                cos(theta), -sin(theta), m_status_module.pose().pose.position.x,
                sin(theta), cos(theta), m_status_module.pose().pose.position.y,
                0.0, 0.0, 1.0
        }};

        auto global_vector{tf::Vector3{m_recovery_module.predictedTargetPosition().x,
                                       m_recovery_module.predictedTargetPosition().y, 1.0}};
        auto local_vector{rotation.inverse() * global_vector};

        auto local_angle_to_goal{atan2(local_vector.y(), local_vector.x())};

        // DEBUG
        ROS_INFO_STREAM("angle to predicted position: " << local_angle_to_goal);

        // re-identify the target if possible
        if (m_tracking_module.target().lastSeen() > 0.2 && m_tracking_module.target().lastSeen() < 10.0
            && std::abs(local_angle_to_goal) < 0.3)
            m_recovery_module.reIdentify(m_tracking_module.target(), m_tracking_module.trackedPersons(),
                                         m_control_module.waypoints(), m_status_module.status());

        // publish prediction markers
        m_visualization_pub.publish(lastPositionMarker());
        m_visualization_pub.publish(predictedPositionMarker());
        m_visualization_pub.publish(predictionRadiusMarker());

        // reset the robot's status to WAITING if the target is lost for more than 10 seconds
    } else {
        m_tracking_module.target() = Person{};
        m_status_module.status() = StatusModule::Status::WAITING;
    }
}


void RobustPeopleFollower::bumperCallback(const kobuki_msgs::BumperEventConstPtr& msg)
{
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
        m_emergency_stop = true;
}


void RobustPeopleFollower::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // save data from message
    auto pose_stamped{geometry_msgs::PoseStamped{}};
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = msg->pose.pose;

    // process data
    m_status_module.processOdometryData(pose_stamped);
}


void RobustPeopleFollower::skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg)
{
    // save data from message
    auto skeleton{body_tracker_msgs::Skeleton{}};
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

    // process data
    m_tracking_module.processSkeletonData(skeleton, m_status_module.pose(), m_status_module.status());
}


void RobustPeopleFollower::publishPersonMarkers() const
{
    auto person_markers{std::vector<visualization_msgs::Marker>{}};
    auto person_vectors{std::vector<visualization_msgs::Marker>{}};

    auto i{0};
    for (const auto& p : *m_tracking_module.trackedPersons()) {
        if (p.distance() > 0) {

            // person marker
            visualization_msgs::Marker marker{};
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "persons";
            marker.id = i;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration{0.3};
            marker.pose.position.x = p.pose().pose.position.x;
            marker.pose.position.y = p.pose().pose.position.y;
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


            // person vector
            auto vector{visualization_msgs::Marker{}};
            vector.header.frame_id = "odom";
            vector.header.stamp = ros::Time::now();
            vector.ns = "vectors";
            vector.id = i;
            vector.type = visualization_msgs::Marker::ARROW;
            vector.action = visualization_msgs::Marker::ADD;
            vector.lifetime = ros::Duration{0.3};
            vector.pose.position.x = p.pose().pose.position.x;
            vector.pose.position.y = p.pose().pose.position.y;
            vector.pose.position.z = 1.3;

            auto q{tf::Quaternion{tf::createQuaternionFromYaw(p.meanAngle())}};
            vector.pose.orientation.x = q.getX();
            vector.pose.orientation.y = q.getY();
            vector.pose.orientation.z = q.getZ();
            vector.pose.orientation.w = q.getW();

            vector.scale.x = VECTOR_LENGTH_FACTOR * p.meanVelocity();
            vector.scale.y = 0.1;
            vector.scale.z = 0.1;

            vector.color.a = 1.0;
            vector.color.r = 1.0;

            person_vectors.emplace_back(vector);


            ++i;
        }
    }

    // publish markers
    for (const auto& m : person_markers)
        m_visualization_pub.publish(m);
    for (const auto& m : person_vectors)
        m_visualization_pub.publish(m);
}


void RobustPeopleFollower::publishWaypoints() const
{
    auto line_strip{visualization_msgs::Marker{}};
    auto line_list{visualization_msgs::Marker{}};

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

    // publish waypoints
    for (const auto& w : *m_control_module.waypoints()) {
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


visualization_msgs::Marker RobustPeopleFollower::lastPositionMarker() const
{
    auto marker{visualization_msgs::Marker{}};
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "last_seen";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration{0.3};
    marker.pose.position.x = m_tracking_module.target().pose().pose.position.x;
    marker.pose.position.y = m_tracking_module.target().pose().pose.position.y;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    marker.mesh_resource = "package://robust_people_follower/meshes/standing.dae";

    return marker;
}


visualization_msgs::Marker RobustPeopleFollower::predictedPositionMarker() const
{
    auto marker{visualization_msgs::Marker{}};
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "estimation";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration{0.3};
    marker.pose.position.x = m_recovery_module.predictedTargetPosition().x;
    marker.pose.position.y = m_recovery_module.predictedTargetPosition().y;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.5;
    marker.color.b = 1.0;
    marker.mesh_resource = "package://robust_people_follower/meshes/standing.dae";

    return marker;
}


visualization_msgs::Marker RobustPeopleFollower::predictionRadiusMarker() const
{
    auto marker{visualization_msgs::Marker{}};
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "radius";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration{0.3};
    marker.pose.position.x = m_recovery_module.predictedTargetPosition().x;
    marker.pose.position.y = m_recovery_module.predictedTargetPosition().y;
    marker.pose.orientation.w = 1.0;

    // scales the diameter, therefore we have to take the radius times two
    marker.scale.x = m_recovery_module.predictionRadius() * 2.0;
    marker.scale.y = m_recovery_module.predictionRadius() * 2.0;

    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.b = 1.0;

    return marker;
}


/**
 * @brief Entry point for the node, runs the main loop.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robust_people_follower");
    RobustPeopleFollower robust_people_follower{ros::this_node::getName()};
    robust_people_follower.runLoop();
    return 0;
}
