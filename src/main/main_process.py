#!/usr/bin/env python3
import rospy
import keyboard

from obtain_pcd.srv import ObtainPcd
from halcon_package.srv import RegistratePose
from pose_transformation.srv import PoseTransform
from move_robot.srv import MoveToPose

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

def move_robot_to_pose_client(x, y, z):
    rospy.wait_for_service('move_to_pose_service')
    try:
        move_to_pose = rospy.ServiceProxy('move_to_pose_service', MoveToPose)
        resp1 = move_to_pose(x, y, z)
        return resp1.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

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
            rotation_matrix,pose_g, pose_up = response.rotation_matrix, response.grasp_pose, response.up_pose
            rospy.loginfo("Pose transformation completed !")
        elif keyboard.is_pressed('4'):
            rospy.loginfo("Start grasp !")
            result = move_robot_to_pose_client(rotation_matrix, pose_g, pose_up)
            if result:
                rospy.loginfo("Robot moved successfully!")
            else:
                rospy.loginfo("Failed to move robot.")
        else:
            pass
        
if __name__ == '__main__':
    rospy.init_node('main_process')
    main_process()
    rospy.spin()
