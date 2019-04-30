#include <ros/ros.h>
#include <body_tracker_msgs/Skeleton.h>
#include <geometry_msgs/Twist.h>


// FIXME: now that's ugly (supports only one person anyway).
int bodyId;
int numberOfGestures;
float distanceCenterOfMass;
float centerOfMassY;


/**
 * @brief Sets the global variables to values in the message from the subscribed topic.
 * @param msg the message from the subscribed topic
 */
void skeletonCallback(const body_tracker_msgs::Skeleton::ConstPtr& msg)
{
    bodyId = msg->body_id;
    numberOfGestures = msg->gesture;
    distanceCenterOfMass = msg->centerOfMass.x;
    centerOfMassY = msg->centerOfMass.y;
}


int main(int argc, char** argv)
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

        // FIXME: ugly.
        msg.angular.z = 0;

        if (bodyId != 0 && distanceCenterOfMass > 1300)
            msg.linear.x = 0.2;

        // publish velocity command for moving straight
        pub.publish(msg);

        // TODO: move backwards if person is too close

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}