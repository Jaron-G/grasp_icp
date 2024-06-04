#!/usr/bin/env python3
import pyvista as pv
import numpy as np
import transforms3d as tfs
import rospy
from std_msgs.msg import Int32
import math
import random
from PoseEstimation import PoseEst

def model_transform(model_mesh, transformation):
    x, y, z, rx, ry, rz, _ = transformation

    model_mesh = model_mesh.rotate_z(rz, inplace=False)
    model_mesh = model_mesh.rotate_y(ry, inplace=False)
    model_mesh = model_mesh.rotate_x(rx, inplace=False)
    
    model_mesh = model_mesh.translate([x,y,z])
    
    return model_mesh

def random_pose():
    x, y, z = [random.uniform(-500, 500) for _ in range(3)]
    rx, ry, rz = [random.uniform(-math.pi, math.pi) for _ in range(3)]
    print("The random pose is:", [x, y, z, rx, ry, rz])
    return [x, y, z, rx, ry, rz]

def radians_to_degrees(radians):
    degrees = radians * (180 / math.pi)
    return degrees

def degrees_to_radians(degrees):
    radians = degrees * (math.pi / 180)
    return radians

def msg_callback(msg):
    if msg.data == 3:
        rospy.loginfo("Start point cloud registration")
        pose_design()

def pose_design(pose_g = [0,0,0,0,0,0], pose_o = [0,0,0,0,0,0]):
    #model_names = ['A1', 'A2', 'A3','A6','B4', 'C0','D1','D2', 'D4', 'E0u','E0s', 'E1', 'E1u','E3', 'E3u' , 'F1']
    model_names = ['pipe','E0u',  'E1u', 'E3u' , 'F1','u_joint']
    model_file_path = ['/catkin_ws/src/grasp_icp/pcd/'+name+'.stl' for name in model_names]
    print(model_file_path)
    # Load model
    model_mesh = pv.read(model_file_path[0])
    mat_rot = tfs.euler.euler2mat(pose_g[5],pose_g[4],pose_g[3],'szyx')
    # Create a column vector from the last three elements of the first sublist of transformations_scaled
    translation_vector = np.array([[pose_g[0]], 
                                [pose_g[1]], 
                                [pose_g[2]]])

    # Horizontally stack the rotation matrix with the translation vector
    mat = np.hstack((mat_rot, translation_vector))

    # Add a row [0, 0, 0, 1] to the end
    final_matrix = np.vstack((mat, np.array([0, 0, 0, 1])))
    print("The final pose matrix is", final_matrix)

    # Apply transformation to the model
    rx_o, ry_o, rz_o = map(radians_to_degrees, pose_o[3:6])
    
    model_trans = model_transform(model_mesh, [pose_o[0], pose_o[1], pose_o[2], rx_o, ry_o, rz_o, 0])

    # Load and transform gripper
    gripper_path = "/catkin_ws/src/grasp_icp/pcd/gripper.stl"
    gripper_mesh = pv.read(gripper_path)
    rx_g, ry_g, rz_g = map(radians_to_degrees, pose_g[3:6])
    print("the angle in rad is:",pose_g[3:6])
    print("the angle in degree is:",rx_g, ry_g, rz_g)
    gripper_trans = model_transform(gripper_mesh, [pose_g[0], pose_g[1], pose_g[2], rx_g, ry_g, rz_g, 0])

    # Create a Plotter instance and add meshes
    plotter = pv.Plotter()
    plotter.add_mesh(model_trans, color='red', show_edges=True)
    plotter.add_mesh(gripper_trans, color='lightblue', show_edges=True)

    # Add axes for reference
    axes = pv.Axes(show_actor=True, actor_scale=200.0, line_width=5)
    axes.origin = (0, 0, 0)
    plotter.add_actor(axes.actor)

    # Show the plot
    plotter.show()
    return final_matrix

def main():
    pose_0 = random_pose()
    # final_matrix = pose_design(pose_0, pose_0)
    final_matrix =  pose_design()
    rotation_matrix, pose_g, pose_up = PoseEst('pipe', final_matrix)
    final_matrix = pose_design(pose_g)

if __name__ == '__main__':
    main()
