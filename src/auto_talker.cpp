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

    while (ros::ok()) {
        for (int i = 1; i <= 1000; ++i) {
                std_msgs::Int32 msg;
                ros::Duration(3).sleep();
                msg.data = 7;
                int_chatter_pub.publish(msg);
                ros::Duration(5).sleep();
                msg.data = 3;
                int_chatter_pub.publish(msg);
                ros::Duration(20).sleep();

                // Switch case for printing corresponding contents
                switch (msg.data) {
                    case 7:
                        std::cout << "7: Camera & Projector" << std::endl;
                        break;
                    case 3:
                        std::cout << "3: Main_Process" << std::endl;
                        break;
                    default:
                        std::cout << "Unknown command" << std::endl;
                }

                ROS_INFO("Published: %d", msg.data);

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
    return 0;
}
