#!/usr/bin/env python3
import rospy
import numpy as np
import math
import transforms3d as tfs

def r_t_to_homogeneous_matrix(R, T):
    R1 = np.vstack([R, np.array([0, 0, 0])])
    T1 = np.vstack([T, np.array([1])])
    HomoMtr = np.hstack([R1, T1])
    return HomoMtr

def homogeneous_matrix_to_r_t(HomoMtr):
    R = HomoMtr[:3, :3]
    T = HomoMtr[:3, 3]
    return R, T

def rotation_matrix_to_axis_angle(R):
    assert R.shape == (3, 3)
    angle = math.acos((np.trace(R) - 1) / 2)
    r = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])
    axis = r / (2 * math.sin(angle))
    # Scale axis by the angle to get the vector with magnitude equal to the angle
    axis_with_angle = axis * angle
    return axis_with_angle

def generate_pose(homo_matrix):
        tempR, tempT = homogeneous_matrix_to_r_t(homo_matrix)
        euler_angles = [0,0,0]
        euler_angles[2],euler_angles[1],euler_angles[0] = tfs.euler.mat2euler(tempR, axes='szyx')
        pose_euler = np.hstack([tempT.flatten(), euler_angles])
        return pose_euler, tempR

def PoseTrans(matched_item, final_pose):
        # Coordinate transformation: camera coordinate system transformation to robot coordinate system transformation
        trans_matrix = np.loadtxt('/catkin_ws/src/grasp_icp/config/matrix.txt')
        final_trans_pose = np.matmul(trans_matrix, final_pose)

        # Get model configuration parameters from the parameter server
        matched_model_name = "/pose_parameters/" + matched_item
        matched_model_config = rospy.get_param(matched_model_name)
        t_vec = np.array(matched_model_config['t_vec'])
        t_vec_up = np.array(matched_model_config['t_vec_up'])
        r_vec = np.array(matched_model_config['r_vec'], dtype=np.float64)

        pR_matrix = tfs.euler.euler2mat(r_vec[2],r_vec[1],r_vec[0],'szyx')
        pose_matrix = r_t_to_homogeneous_matrix(pR_matrix, t_vec)
        up_pose_matrix = r_t_to_homogeneous_matrix(pR_matrix, t_vec_up)

        # Change grasping pose based on grasping configuration
        pose_trans_matrix = np.matmul(final_trans_pose, pose_matrix)
        up_pose_trans_matrix = np.matmul(final_trans_pose, up_pose_matrix)

        grasp_pose_euler, tempR= generate_pose(pose_trans_matrix)
        up_pose_euler, tempR = generate_pose(up_pose_trans_matrix)

        return tempR, grasp_pose_euler, up_pose_euler

if __name__ == "__main__":
    rospy.init_node('pose_tranformation_node')
