#include <stdlib.h>

#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


#define WAITING 0
#define FOLLOWING 1
#define SEARCHING 2


// ####### DATA STRUCTURES #######

/**
 * Stores a position in 2-dimensional space.
 */
struct Position2D
{
    double x;
    double y;
};


/**
 * Stores an orientation in 2-dimensional space.
 */
struct Orientation
{
    double z;
    double w;
};


/**
 * Stores data related to the skeleton info of a person.
 */
struct SkeletonInfo
{
    int bodyId;
    int numberOfGestures;
    float distanceCenterOfMass;
    float centerOfMassY;
};


/**
 * Stores data related to the roboter.
 */
class Turtlebot
{
public:
    int status;
    Position2D position;
    Orientation orientation;


    Turtlebot()
    {
        status = WAITING;
        position.x = position.y = 0;
        orientation.z = orientation.w = 0;
    }
};


/**
 * Stores data related to a tracked person.
 */
class Person
{
public:
    bool isTracked;
    bool isTarget;
    SkeletonInfo skeleton;
    Position2D position;

    // TODO: calculate velocity
    double velocity;

    ros::Time gestureBegin;


    Person(SkeletonInfo skeleton, Position2D position)
    {
        isTracked = true;
        isTarget = false;
        this->skeleton = skeleton;
        this->position = position;
        velocity = 0;
        gestureBegin = ros::Time(0);
    }
};


// ####### GLOBAL VARIABLES #######

// instance of the turtlebot robot
Turtlebot turtlebot;

// list of persons in the frame
std::vector<Person> trackedPersons;

// stores the path of the robot
nav_msgs::Path robotPath;

// sequence number for path
uint32_t seq = 0;


// TODO: replace with path following algorithm
// distance and deviation of target
float targetDistance = 0;
float targetYDeviation = 0;


// ####### FUNCTION PROTOTYPES #######

void debugPrintout();

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr &msg);


// ####### ENTRY POINT #######

int main(int argc, char **argv)
{
    ros::init(argc, argv, "body_tracker");
    ros::NodeHandle nh;

    ros::Subscriber odomSub = nh.subscribe("/odom", 10, odometryCallback);
    ros::Subscriber skeletonSub = nh.subscribe("/body_tracker/skeleton", 10, skeletonCallback);

    ros::Publisher velocityPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("/path", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        debugPrintout();

        // move the robot
        // message to store velocity commands in
        geometry_msgs::Twist msg;

        // TODO: make smoother
        if (targetYDeviation < -100) {
            msg.angular.z = -0.5;
            ROS_INFO("[TURNING LEFT at %f]", msg.angular.z);
        } else if (targetYDeviation > 100) {
            msg.angular.z = 0.5;
            ROS_INFO("[TURNING RIGHT at %f]", msg.angular.z);
        } else
            msg.angular.z = 0;

        // publish velocity command for rotation
        velocityPub.publish(msg);

        if (targetDistance > 1800) {
            msg.linear.x = 0.2;
            ROS_INFO("[MOVING FORWARDS at %f]", msg.linear.x);
        } else if (targetDistance < 1000 && targetDistance > 0) {
            msg.linear.x = -0.2;
            ROS_INFO("[MOVING BACKWARDS at %f]", msg.linear.x);
        }

        // publish velocity command for moving straight
        velocityPub.publish(msg);

        // TODO: proper list management, testing
        if (!trackedPersons.empty()) {
            for (std::vector<Person>::iterator iter = trackedPersons.begin(); iter != trackedPersons.end(); ++iter) {
                if (iter->isTracked && iter->skeleton.distanceCenterOfMass == 0)
                    trackedPersons.erase(iter);
            }
        }

        robotPath.header.seq = seq;
        robotPath.header.stamp = ros::Time::now();
        robotPath.header.frame_id = "odom";

        pathPub.publish(robotPath);
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

    ROS_INFO("Turtlebot information:");
    ROS_INFO("  position:");
    ROS_INFO("    x: %f", turtlebot.position.x);
    ROS_INFO("    y: %f", turtlebot.position.y);
    ROS_INFO("  orientation:");
    ROS_INFO("    z: %f", turtlebot.orientation.z);
    ROS_INFO("    w: %f\n", turtlebot.orientation.w);

    ROS_INFO("target information:");
    ROS_INFO("  distance: %f", targetDistance);
    ROS_INFO("  y-deviation: %f\n", targetYDeviation);

    ROS_INFO("list size: %lu\n", trackedPersons.size());

    if (!trackedPersons.empty()) {
        for (std::vector<Person>::iterator iter = trackedPersons.begin(); iter != trackedPersons.end(); ++iter) {
            if (iter->isTracked) {
                ROS_INFO("id: %d, distance; %f,is target: %d, number of gestures: %d",
                         iter->skeleton.bodyId, iter->skeleton.distanceCenterOfMass, iter->isTarget,
                         iter->skeleton.numberOfGestures);
                ROS_INFO("  is target: %d", iter->isTarget);
                ROS_INFO("  position (relative):");
                ROS_INFO("    x: %f", iter->position.x);
                ROS_INFO("    y: %f", iter->position.y);

                // TODO: calculate correct absolute position
                ROS_INFO("  position (absolute):");
                ROS_INFO("    x: %f", iter->position.x);
                ROS_INFO("    y: %f", iter->position.y);

                ROS_INFO("  number of gestures: %d", iter->skeleton.numberOfGestures);
                ROS_INFO("  gesture begin time: %d", iter->gestureBegin.sec);
                ROS_INFO("  distance: %f", iter->skeleton.distanceCenterOfMass);
                ROS_INFO("  y-deviation of center of mass: %f\n", iter->skeleton.centerOfMassY);
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
    turtlebot.position.x = msg->pose.pose.position.x;
    turtlebot.position.y = msg->pose.pose.position.y;
    turtlebot.orientation.z = msg->pose.pose.orientation.z;
    turtlebot.orientation.w = msg->pose.pose.orientation.w;

    geometry_msgs::PoseStamped pose;
    pose.header.seq = seq;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "odom";

    pose.pose.position.x = msg->pose.pose.position.x;
    pose.pose.position.y = msg->pose.pose.position.y;
    pose.pose.position.z = msg->pose.pose.position.z;

    pose.pose.orientation.z = msg->pose.pose.orientation.z;
    pose.pose.orientation.w = msg->pose.pose.orientation.w;

    robotPath.poses.push_back(pose);
}


/**
 * @brief Manages the list of tracked persons with data received from the skeleton topic.
 * @param msg the message from the subscribed topic
 */
void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr &msg)
{

    // save data from message
    SkeletonInfo skeleton = {msg->body_id, msg->gesture, msg->centerOfMass.x, msg->centerOfMass.y};
    Position2D position = {msg->joint_position_spine_mid.x, msg->joint_position_spine_mid.y};

    bool found = false;
    for (std::vector<Person>::iterator iter = trackedPersons.begin(); iter != trackedPersons.end(); ++iter) {
        if (iter->skeleton.bodyId == skeleton.bodyId) {
            found = true;
            if (iter->isTarget) {
                iter->skeleton = skeleton;
                iter->position = position;
                targetDistance = skeleton.distanceCenterOfMass;
                targetYDeviation = skeleton.centerOfMassY;
            } else {

                // update information
                iter->skeleton = skeleton;
                iter->position = position;

                // FIXME
                // check for gestures
                if (skeleton.numberOfGestures == 2) {
                    if (iter->gestureBegin.sec == 0) {
                        iter->gestureBegin = ros::Time::now();
                    }
                } else {
                    if (iter->gestureBegin.sec != 0) {

                        // TODO: figure out how to play a sound
                        // new target selected after 3 seconds of closing both hands
                        if (ros::Time::now().sec - iter->gestureBegin.sec >= 3) {
                            iter->isTarget = true;
                            turtlebot.status = FOLLOWING;

                            // reset gesture beginning time
                            iter->gestureBegin = ros::Time(0);
                        }
                    }
                }
            }
        }
    }

    // save new person if new id has been detected
    if (!found) {
        trackedPersons.push_back(Person(skeleton, position));
    }
}
