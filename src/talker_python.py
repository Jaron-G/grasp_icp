#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def int_talker():
    rospy.init_node('int_talker', anonymous=True)
    int_chatter_pub = rospy.Publisher('chatter', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        user_input = input("What to start?(1:Camera & Projector, 2:Reconstruction, 3:ICP, 4:Coord Transformation): ")
        try:
            user_input = int(user_input)
        except ValueError:
            rospy.logwarn("Invalid input. Please enter an integer (1, 2, 3, or 4).")
            continue

        msg = Int32()
        msg.data = user_input

        int_chatter_pub.publish(msg)
        rospy.loginfo("Published: %d", msg.data)

        rate.sleep()

if __name__ == '__main__':
    try:
        int_talker()
    except rospy.ROSInterruptException:
        pass
