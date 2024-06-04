#!/usr/bin/env python3
import halcon as ha
import pyvista as pv
import numpy as np
import transforms3d as tfs
import math
import pcl
 
def convert_pcd_to_ply(ply_file, pcd_file):
    # 读取PCD文件,转换单位m to mm
    point_cloud = pcl.load(pcd_file)
    pt_m = point_cloud.to_array()
    pt_mm = pt_m*1000
    point_cloud = pcl.PointCloud(pt_mm)
    # 保存为Ply文件
    #point_cloud.to_file(ply_file)
    pcl.save(point_cloud,ply_file)

def model_transform(model_mesh, transformation):
    x, y, z, rx, ry, rz, _ = transformation
    model_mesh = model_mesh.rotate_z(rz, inplace=False)
    model_mesh = model_mesh.rotate_y(ry, inplace=False)
    model_mesh = model_mesh.rotate_x(rx, inplace=False)
    model_mesh = model_mesh.translate([x,y,z])
    return model_mesh

def radians_to_degrees(radians):
    degrees = radians * (180 / math.pi)
    return degrees

def degrees_to_radians(degrees):
    radians = degrees * (math.pi / 180)
    return radians
        
def Registration(pose_g = [0,0,0,0,0,0]):
    # Define the model name
    #model_names = ['A1', 'A2', 'A3','A6','B4','D2', 'D4','E0u',  'E1u', 'E3u' , 'F1','u_joint']
    #model_names = ['bowl','E0u',  'E1u', 'E3u' , 'F1','u_joint','gear','piston_rod']
    model_names = ['t_pipe_u','pipe_u','L_pipe_u']
    #score_threshold = [0.15, 0.26, 0.35, 0.27, 0.39, 0.4, 0.25, 0.25, 0.45, 0.45, 0.10,0.1]
    score_threshold = [0.3,0.2,0.3]
    # Specify the full path for your STL and PLY files
    model_file_path = []
    for name in model_names:
        path = '/catkin_ws/src/grasp_icp/pcd/'+name+'.stl'
        model_file_path.append(path)
    print(model_file_path)


    # convert_pcd_to_ply
    ply_file_path = '/catkin_ws/src/grasp_icp/pcd/scence_gazebo.ply'  #填入ply文件的路径
    pcd_file_path = '/catkin_ws/src/grasp_icp/pcd/scence_gazebo.pcd'  #填入pcd文件的路径
    convert_pcd_to_ply(ply_file_path, pcd_file_path)
    scene_file_path = '/catkin_ws/src/grasp_icp/pcd/scence_gazebo.ply'

    # Read scene model
    Scene_3d, status = ha.read_object_model_3d(scene_file_path, "mm",[],[])
    print(f"Initial Scene Points: {ha.get_object_model_3d_params(Scene_3d, 'num_points')}")

    # Preprocess the scene 
    ObjectModel3DThresholdedY = ha.select_points_object_model_3d(Scene_3d, 'point_coord_y', -0.6, 0.6)
    print(f"After Y Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedY, 'num_points')}")
    ObjectModel3DThresholdedX = ha.select_points_object_model_3d(ObjectModel3DThresholdedY, 'point_coord_x', -0.6, 0.6)
    print(f"After X Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedX, 'num_points')}")
    ObjectModel3DThresholdedZ = ha.select_points_object_model_3d(ObjectModel3DThresholdedX, 'point_coord_z', 0.6, 1.01)
    print(f"After Z Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedZ, 'num_points')}")

    ObjectModel3DConnected = ha.connection_object_model_3d(ObjectModel3DThresholdedZ, 'distance_3d', 0.0035)
    print(f"ObjectModel3DConnected: {ha.get_object_model_3d_params(ObjectModel3DConnected, 'num_points')}")
    ObjectModel3DSelected = ha.select_object_model_3d(ObjectModel3DConnected, 'num_points', 'and', 200, 30000)
    print(f"ObjectModel3DSelected: {ha.get_object_model_3d_params(ObjectModel3DSelected, 'num_points')}")
    UnionObjectModel3D = ha.union_object_model_3d(ObjectModel3DSelected, 'points_surface')
    print(f"UnionObjectModel3D: {ha.get_object_model_3d_params(UnionObjectModel3D, 'num_points')}")
    TargetPC, Information = ha.triangulate_object_model_3d(UnionObjectModel3D, 'greedy', [], [])
    print(f"TargetPC: {ha.get_object_model_3d_params(TargetPC, 'num_points')}")
    
    # Read object model
    model_mesh = []
    for path in model_file_path:
        Object_3d, status = ha.read_object_model_3d(path, "mm",[],[])
        # Create object surface template
        Object_surface = ha.create_surface_model(Object_3d, 0.05, 'train_view_based', 'true') # This line would require access to display, and cannot be run on a remote server
        model_mesh.append(Object_surface)
    # Object_surface = ha.create_surface_model(Object_3d, 0.05, [], [])

    match_score = []
    match_pose = []
    for mesh, thre in zip(model_mesh, score_threshold):
        # Matching without view based
        pose, score, ID = ha.find_surface_model(mesh, TargetPC, 0.05, 0.2, thre, "true", ['num_matches', 'use_view_based'], [1, 'false'])
        print("score is:", score)
        # Only append score[0] if score is not empty and score[0] is a number
        if score and isinstance(score[0], (float, int)):
            match_score.append(score[0])
            match_pose.append(pose)
        else:
            print("No valid score found to append.")
            match_score.append(0)
            
    # Print results   
    transformations = [match_pose[i:i+6] for i in range(0, len(match_pose), 7)]
    print(transformations)   
    #highest_score = [0.31, 0.6,0.55, 0.44,0.48, 0.57, 0.34, 0.45,  0.69, 0.63, 0.23]
    highest_score = [0.31, 0.2,0.2]
    normalized_score = []
    for real_score, max_score in zip(match_score, highest_score):
        norm_score = real_score/max_score
        normalized_score.append(norm_score)
    print('The normalized scores are ', normalized_score)
    max_index1 = np.argmax(normalized_score)
    model_file_path_vis = model_file_path[max_index1]
    print('The matched item is: ', model_names[max_index1])
    matched_model = model_names[max_index1]
    filtered_list = [element for element in normalized_score if element != 0]
    print('The list removed all 0 is: ', filtered_list)
    scaled_trans = [[element * 1000 if index < 3 else element for index, element in enumerate(sublist)] for sublist in transformations[0]]
    print('The scaled pose is ', scaled_trans)
    max_index2 = np.argmax(filtered_list)
    print('The matched pose is ', scaled_trans[max_index2])
    rot_deg = scaled_trans[max_index2][3:6]
    print("the rotation angles in degree are", rot_deg)

    # Convert degree into radians
    rot_rad = []
    for i in rot_deg:
        d = degrees_to_radians(i)
        rot_rad.append(d)
    print("the rotation angles in radians are", rot_rad)

    mat_rot = tfs.euler.euler2mat(rot_rad[2],rot_rad[1],rot_rad[0],'szyx')
    # Create a column vector from the last three elements of the first sublist of transformations_scaled
    translation_vector = np.array([[scaled_trans[max_index2][0]], 
                                [scaled_trans[max_index2][1]], 
                                [scaled_trans[max_index2][2]]])

    # Horizontally stack the rotation matrix with the translation vector
    mat = np.hstack((mat_rot, translation_vector))
    # Add a row [0, 0, 0, 1] to the end
    final_matrix = np.vstack((mat, np.array([0, 0, 0, 1])))
    print("The final pose matrix is", final_matrix)
     
    stl_meshes = []  
    # Load STL
    for i in range(1):
        stl_meshes.append(pv.read(model_file_path_vis))

    # load scene
    ply_mesh = pv.read(scene_file_path)
    
    stl_meshes_trans=[model_transform(stl_meshes[i], scaled_trans[max_index2]) for i in range(1)]

    # Create a Plotter instance
    plotter = pv.Plotter()

    # load gripper coordination
    # Load STL1
    gripper_path = "/catkin_ws/src/grasp_icp/pcd/gripper.stl"
    gripper_mesh = pv.read(gripper_path)
    print("The gripper pose is:", pose_g)
    rx1 = radians_to_degrees(pose_g[3])
    ry1 = radians_to_degrees(pose_g[4])
    rz1 = radians_to_degrees(pose_g[5])
    gripper_trans = model_transform(gripper_mesh, [pose_g[0],pose_g[1],pose_g[2],rx1, ry1, rz1, 0])

    # Add  meshes to the plotter
    plotter.add_mesh(ply_mesh, color='lightblue', show_edges=True)
    for i in range(1):
        plotter.add_mesh(stl_meshes_trans[i], color='red', show_edges=True)
    plotter.add_mesh(gripper_trans, color='lightblue', show_edges=True)
    axes = pv.Axes(show_actor=True, actor_scale=200.0, line_width=5)
    axes.origin = (0, 0, 0)
    plotter.add_actor(axes.actor)
    plotter.show()
    return matched_model, final_matrix
    
def main():
    #读场景，模型，配准，return
    matched_model, final_matrix = Registration()
    print('matched_model:{}'.format(matched_model))
    print('final_matrix:{}'.format(final_matrix))
    
if __name__ == '__main__':
    main()
