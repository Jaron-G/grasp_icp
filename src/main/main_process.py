#!/usr/bin/env python3
import rospy
import std_msgs
import keyboard
# from halcon_registration import Registration
# from PoseTransformation import PoseTrans
from grasp import moverobot
import numpy as np

from halcon_package.srv import * 
from pose_transformation.srv import PoseTransform,PoseTransformResponse

def registration_client():
    rospy.wait_for_service('registration')
    try:
        registration = rospy.ServiceProxy('registration', RegistratePose)
        resp1 = registration()
        return resp1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

def pose_transformation_client(x, y):
    rospy.wait_for_service('pose_transformation')
    try:
        pose_transformation = rospy.ServiceProxy('pose_transformation', PoseTransform)
        resp1 = pose_transformation(x, y)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main_process():
    rospy.init_node('main_process')
    pub = rospy.Publisher('obtain_pcd', std_msgs.msg.String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        if keyboard.is_pressed('1'):
            pub.publish('1')
            rospy.loginfo("Obtain pointcloud !")
        elif keyboard.is_pressed('2'):
            rospy.loginfo("Start perform registration !")
            response = registration_client()
            final_pose, matched_model = response.final_pose,response.matched_model
            rospy.loginfo("Registration completed !")
        elif keyboard.is_pressed('3'):
            rospy.loginfo("Pose transformation !")
            response = pose_transformation_client(final_pose, matched_model)
            rotation_matrix,pose_g, pose_up = np.array(response.rotation_matrix).reshape(3,3), np.array(response.grasp_pose), np.array(response.up_pose)
            rospy.loginfo("Pose transformation completed !")
        elif keyboard.is_pressed('4'):
            rospy.loginfo("Start grasp !")
            moverobot(rotation_matrix,pose_g, pose_up, frame_id="base_link")
            rospy.loginfo("Grasp completed !")
        else:
            pass
        rate.sleep()
        
if __name__ == '__main__':
    main_process()
    rospy.spin()
