#!/usr/bin/env python3

'''
This script is the main loop of the real-world grasping experiments
'''

import rospy
import transforms3d
import numpy as np
import rospy
from std_msgs.msg import Int32

import geometry_msgs.msg

from gripper2F85_control import gripper_2F85

from ur_control import UR_robot

def msg_callback(msg):
    if msg.data == 5:
        rospy.loginfo("Moving robot")
        moverobot()

def construct_pose(position, angle, frame_id = "base_link") -> geometry_msgs.msg.Pose:

    # pose_under_camera = function(x, y)
    robot_pose = geometry_msgs.msg.Pose()
    robot_pose.position.x = position[0]
    robot_pose.position.y = position[1]
    robot_pose.position.z = position[2]

    ## transform Euler angles into Quternion, angle = (rx, ry, rz)
    quaternion = transforms3d.euler.euler2quat(angle[0], angle[1], angle[2], 'rxyz')

    robot_pose.orientation.w = quaternion[0]
    robot_pose.orientation.x = quaternion[1]
    robot_pose.orientation.y = quaternion[2]
    robot_pose.orientation.z = quaternion[3]

    # print(transforms3d.euler.quat2euler((0, 7.07109031e-01, 7.07104531e-01, 0), 'rxyz'))
    return robot_pose
def moverobot(rotation_matrix, pose_grasp, pose_pre_grasp, frame_id = "base_link") -> geometry_msgs.msg.Pose:

    base2baselink =  np.array([[ 1,  0, 0],
                            [0,  1,  0],
                            [0, 0, 1]])
    ## Construct robotiq_controller
    gripper = gripper_2F85()
    gripper.open()
    
    ur = UR_robot()

    # Grasping angles
    rot_mat = np.matmul(base2baselink, rotation_matrix)
    rot_angle = transforms3d.euler.mat2euler(rot_mat, 'rxyz')

    table_to_base_offset_x=200
    table_to_base_offset_y=1140
    table_to_base_offset_z=1000
    offset=[table_to_base_offset_x, table_to_base_offset_y, table_to_base_offset_z]
    # Home pose 
    ##更改符号
    #home_position = [-pose_grasp[0]/1000,-pose_grasp[1]/1000,pose_grasp[2]/1000]
    home_position = [(pose_grasp[0]+offset[0])/1000,(pose_grasp[1]+offset[1])/1000,(pose_grasp[2]+offset[2])/1000]
    home_pose = construct_pose(home_position[0:3], rot_angle[0:3], frame_id='base_link')

    # Up pose
    up_position = [(pose_pre_grasp[0]+offset[0])/1000,(pose_pre_grasp[1]+offset[1])/1000,(pose_pre_grasp[2]+offset[2])/1000]
    up_pose = construct_pose(up_position[0:3], rot_angle[0:3], frame_id='base_link')

    # Drop pose
    rot_mat2 = np.array([[ -0.8426062,  0.5372024, -0.0377938],
                            [0.5374385,  0.8432911,  0.0044704],
                            [0.0342727, -0.0165450, -0.9992756]])
    drop_mat = np.matmul(base2baselink, rot_mat2)
    drop_angle = transforms3d.euler.mat2euler(drop_mat, 'rxyz')
    drop_position = [0.66,1.15,1.38]
    drop_pose = construct_pose(drop_position[0:3], drop_angle[0:3], frame_id='base_link')

    ur.go_to_cartesian_pose(up_pose)
    ur.go_to_cartesian_pose(home_pose)
    rospy.sleep(1)
    gripper.close()
    rospy.sleep(1)
    ur.go_to_cartesian_pose(up_pose)
    ur.go_to_cartesian_pose(drop_pose)
    gripper.open()
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('robot', anonymous=True)
    rospy.Subscriber("chatter", Int32, msg_callback)
    rospy.spin()
