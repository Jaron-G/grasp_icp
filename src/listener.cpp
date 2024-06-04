#include "ros/ros.h"
#include "std_msgs/Int32.h"

void intCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Received: %d", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "int_listener");
    ros::NodeHandle n;

    // Subscribe to the 'int_chatter' topic and specify the callback function
    ros::Subscriber sub = n.subscribe("chatter", 10, intCallback);

    ros::spin(); // Keep the node running to receive messages

    return 0;
}
