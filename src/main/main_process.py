#!/usr/bin/env python3
import rospy
import keyboard

from grasp import moverobot
import numpy as np

from obtain_pcd.srv import * 
from halcon_package.srv import * 
from pose_transformation.srv import PoseTransform,PoseTransformResponse

def obtain_pcd_client():
    rospy.wait_for_service('obtain_pcd')
    try:
        obtain_pcd = rospy.ServiceProxy('obtain_pcd', ObtainPcd)
        resp1 = obtain_pcd()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

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
    while not rospy.is_shutdown():
        if  keyboard.is_pressed('1'):
            rospy.loginfo("Start btain pointcloud !")
            response = obtain_pcd_client()
            rospy.loginfo("Obtain pointcloud completed!")
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
        
if __name__ == '__main__':
    rospy.init_node('main_process')
    main_process()
    rospy.spin()
