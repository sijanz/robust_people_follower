#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>


// FIXME: now that's ugly (supports only one person anyway). DEPRECATED - substitute with new data structure!
int bodyId;
int numberOfGestures;
float distanceCenterOfMass;
float centerOfMassY;


// ### DATA STRUCTURES ###

/**
 * Stores a position in 2-dimensional space.
 */
struct Position2D
{
    double x;
    double y;
};


/**
 * Store a pose in 2-dimensional space.
 */
struct Pose
{
    double x;
    double y;
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
    Position2D position;
    Pose pose;


    Turtlebot(Position2D position, Pose pose) : position(position), pose(pose)
    {};
};


/**
 * Stores data related to a tracked person.
 */
class Person
{
public:
    SkeletonInfo skeleton;
    Position2D position;
    Pose pose;
    double velocity;


    // TODO: calculate other members via odometry data
    Person(SkeletonInfo skeleton) : skeleton(skeleton)
    {};
};


// list of persons in the frame
std::list<Person> trackedPersons;


/**
 * @brief Sets the global variables to values in the message from the subscribed topic.
 * @param msg the message from the subscribed topic
 */
void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr &msg)
{

    // TODO: select new ID when gesture is done for at least 3 seconds
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "body_tracker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("body_tracker/skeleton", 10, skeletonCallback);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        // DEBUG
        ROS_INFO("body id: %d", bodyId);
        ROS_INFO("number of gestures: %d", numberOfGestures);
        ROS_INFO("distance to center of mass: %f", distanceCenterOfMass);
        ROS_INFO("y-deviation of center of mass: %f\n", centerOfMassY);

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
