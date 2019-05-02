#include <stdlib.h>

#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


// FIXME: now that's ugly (supports only one person anyway). DEPRECATED - substitute with new data structure!
int bodyId;
int numberOfGestures;
float distanceCenterOfMass;
float centerOfMassY;


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
    Position2D position{};
    Orientation orientation{};


    Turtlebot()
    {
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
    SkeletonInfo skeleton{};
    Position2D position{};
    double velocity;


    Person()
    {
        skeleton.bodyId = skeleton.numberOfGestures = 0;
        skeleton.centerOfMassY = skeleton.distanceCenterOfMass = 0;
        position.x = position.y = 0;
        velocity = 0;
    }
};


// ####### FUNCTION PROTOTYPES #######


void debugPrintout();

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr &msg);


// ####### GLOBAL VARIABLES #######


// instance of the turtlebot robot
Turtlebot turtlebot;

// list of persons in the frame
std::vector<Person> trackedPersons;

// instance of target
Person target;


// ####### ENTRY POINT #######


int main(int argc, char **argv)
{
    ros::init(argc, argv, "body_tracker");
    ros::NodeHandle n;

    ros::Subscriber odomSub = n.subscribe("/odom", 10, odometryCallback);
    ros::Subscriber skeletonSub = n.subscribe("/body_tracker/skeleton", 10, skeletonCallback);

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        debugPrintout();

        // message to store velocity commands in
        geometry_msgs::Twist msg;

        if (centerOfMassY < -100) {
            msg.angular.z = -0.5;
            ROS_INFO("turning left!");
        } else if (centerOfMassY > 100) {
            msg.angular.z = 0.5;
            ROS_INFO("turning right!");
        } else
            msg.angular.z = 0;

        // publish velocity command for rotation
        pub.publish(msg);

        if (bodyId != 0 && distanceCenterOfMass > 1300)
            msg.linear.x = 0.2;
        else if (bodyId != 0 && distanceCenterOfMass < 1000 && distanceCenterOfMass > 0)
            msg.linear.x = -0.2;

        // publish velocity command for moving straight
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


void debugPrintout()
{
    system("clear");

    ROS_INFO("Turtlebot info:");
    ROS_INFO("  position:");
    ROS_INFO("    x: %f", turtlebot.position.x);
    ROS_INFO("    y: %f", turtlebot.position.y);
    ROS_INFO("  orientation:");
    ROS_INFO("    z: %f", turtlebot.orientation.z);
    ROS_INFO("    w: %f\n", turtlebot.orientation.w);

    ROS_INFO("tracked persons: %lu\n", trackedPersons.size());
    if (!trackedPersons.empty()) {
        for (std::vector<Person>::iterator iter = trackedPersons.begin(); iter != trackedPersons.end(); ++iter) {
            ROS_INFO("id: %d", iter->skeleton.bodyId);
            ROS_INFO("  position (relative):");
            ROS_INFO("    x: %f", iter->position.x);
            ROS_INFO("    y: %f", iter->position.y);
            ROS_INFO("  position (absolute):");
            ROS_INFO("    x: %f", iter->position.x);
            ROS_INFO("    y: %f", iter->position.y);
            ROS_INFO("  number of gestures: %d", iter->skeleton.numberOfGestures);
            ROS_INFO("  distance: %f", iter->skeleton.distanceCenterOfMass);
            ROS_INFO("  y-deviation of center of mass: %f\n", iter->skeleton.centerOfMassY);
        }
    }

    ROS_INFO("target id: %d", target.skeleton.bodyId);
}


void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    turtlebot.position.x = msg->pose.pose.position.x;
    turtlebot.position.y = msg->pose.pose.position.y;
    turtlebot.orientation.z = msg->pose.pose.orientation.z;
    turtlebot.orientation.w = msg->pose.pose.orientation.w;
}


/**
 * @brief Sets the global variables to values in the message from the subscribed topic.
 * @param msg the message from the subscribed topic
 */
void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr &msg)
{

    // TODO: select new ID when gesture is done for at least 3 seconds
    // TODO: play sound
    if (msg->gesture == 2) {
        bodyId = msg->body_id;

        // DEBUG
        ROS_INFO("bodyId to track: %d", bodyId);
    }

    if (msg->body_id == bodyId) {
        numberOfGestures = msg->gesture;
        distanceCenterOfMass = msg->centerOfMass.x;
        centerOfMassY = msg->centerOfMass.y;
    }
}
