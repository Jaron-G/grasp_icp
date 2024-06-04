#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

from halcon_registration import Registration
from PoseTransformation import PoseTrans
from grasp import moverobot

def msg_callback(msg):
    if msg.data == 3:
        rospy.loginfo("Robot Grasping")
        main()

def main():
    matched_model, final_matrix = Registration()
    rotation_matrix, pose_g, pose_up = PoseTrans(matched_model, final_matrix)
    moverobot(rotation_matrix,pose_g, pose_up, frame_id="base_link")

if __name__ == '__main__':
    rospy.init_node('Pose', anonymous=True)
    rospy.Subscriber("chatter", Int32, msg_callback)
    rospy.spin()
