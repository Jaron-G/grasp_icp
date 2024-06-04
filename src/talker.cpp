#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "int_talker");
    ros::NodeHandle n;

    // Create a publisher that publishes messages of type Int32 to the 'chatter' topic
    ros::Publisher int_chatter_pub = n.advertise<std_msgs::Int32>("chatter", 10);

    ros::Rate loop_rate(10);  // 10 Hz (you can adjust the publishing rate)

    while (ros::ok())
    {
        std_msgs::Int32 msg;

        // Get user input as an integer
        int user_input;
        std::cout << "What to start?(1:Camera & Projector, 2:Reconstruction, 3:Registration, 4:Coord Transformation, 5:Robot): ";
        std::cin >> user_input;
        msg.data = user_input;

        // Publish the user's integer
        int_chatter_pub.publish(msg);

        // Log the integer for debugging (optional)
        ROS_INFO("Published: %d", msg.data);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
