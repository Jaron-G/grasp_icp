import open3d as o3d
import numpy as np
import transforms3d as tfs
import rospy
import time

FINGER_WIDTH = 20
FINGER_HEIGHT = 45
FINGER_THICKNESS = 15



HAND_EYE_MATRIX_PATH = '/catkin_ws/src/grasp_icp/config/'

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

def r_t_to_homogeneous_matrix(R, T):
    R1 = np.vstack([R, np.array([0, 0, 0])])
    T1 = np.vstack([T, np.array([1])])
    HomoMtr = np.hstack([R1, T1])
    return HomoMtr


def contact_detect(matched_item, matched_matrix, coincide_num_points: int, offset = 0) -> bool:
    """
    :param matched_matrix: 相机坐标系下配准后的变换矩阵
    :param coincide_num_points: 重合的点数阈值
    :return: 是否发生碰撞
    """
    start = time.time()
    hand_eye_matrix = np.loadtxt('/catkin_ws/src/grasp_icp/config/matrix.txt')
    final_trans_pose = np.matmul(hand_eye_matrix, matched_matrix)

    # t_pipe_u: {t_vec: [[14], [20], [-165]], t_vec_up: [[14], [20], [-280]], r_vec: [[0], [0], [0]]}
    # pipe_u: {t_vec: [[20], [27.5], [-165]], t_vec_up: [[20], [27.5], [-280]], r_vec: [[0], [0], [1.5708]]}
    # L_pipe_u: {t_vec: [[60], [40], [-165]], t_vec_up: [[60], [40], [-280]], r_vec: [[0], [0], [0]]}


    # Get model configuration parameters from the parameter server
    matched_model_name = "/pose_parameters/" + matched_item
    matched_model_config = rospy.get_param(matched_model_name)
    t_vec = np.array(matched_model_config['t_vec'])
    t_vec_up = np.array(matched_model_config['t_vec_up'])
    r_vec = np.array(matched_model_config['r_vec'], dtype=np.float64)
    t_vec[0][0] = t_vec[0][0] + offset
    t_vec_up[0][0] = t_vec_up[0][0] + offset

    # 得到夹爪的基坐标系下位姿
    # t_vec = np.array([[45], [260], [30]])
    # r_vec = np.array([[1.5708], [0], [0]], dtype=np.float64)
    pR_matrix = tfs.euler.euler2mat(r_vec[2], r_vec[1], r_vec[0], 'szyx')
    pose_matrix = r_t_to_homogeneous_matrix(pR_matrix, t_vec)
    pose_trans_matrix = np.matmul(final_trans_pose, pose_matrix)
    up_pose_matrix = r_t_to_homogeneous_matrix(pR_matrix, t_vec_up)
    up_pose_trans_matrix = np.matmul(final_trans_pose, up_pose_matrix)

    # 将点云变换到夹爪坐标系下
    scene_point_cloud = o3d.io.read_point_cloud("/catkin_ws/src/reconstruction/output/scene.ply", remove_nan_points=True,
                                                remove_infinite_points=True)  # 原始点云
    scene_point_cloud.transform(hand_eye_matrix)
    final_trans_pose = np.linalg.inv(pose_trans_matrix)
    scene_point_cloud.transform(final_trans_pose)


    grasp_pose_euler, tempR= generate_pose(pose_trans_matrix)
    up_pose_euler, tempR = generate_pose(up_pose_trans_matrix)

    ##############碰撞测试##############
    # translation_vector = np.array([0, 0, -30])
    # scene_point_cloud.translate(translation_vector)
    ###############碰撞测试##############


    # 为夹爪两个fingers建立包围盒并对场景点云进行裁剪和可视化
    bbox1 = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-FINGER_WIDTH / 2, -85 / 2 - FINGER_THICKNESS / 2, 165),
                                                max_bound=(
                                                FINGER_WIDTH / 2, -85 / 2 + FINGER_THICKNESS / 2, 165 + FINGER_HEIGHT))
    bbox2 = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-FINGER_WIDTH / 2, 85 / 2 - FINGER_THICKNESS / 2, 165),
                                                max_bound=(
                                                FINGER_WIDTH / 2, 85 / 2 + FINGER_THICKNESS / 2, 165 + FINGER_HEIGHT))
    bbox1.color = [1, 0, 0]
    bbox2.color = [1, 0, 0]

    cropped_pcd1 = scene_point_cloud.crop(bbox1)
    cropped_pcd2 = scene_point_cloud.crop(bbox2)
    cropped_pcd1.paint_uniform_color([1, 0, 0])
    cropped_pcd2.paint_uniform_color([1, 0, 0])

    num_contact_points = len(cropped_pcd1.points) + len(cropped_pcd2.points)
    print("重合的点云数量：", num_contact_points)

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=200, origin=[0, 0, 0])

    end = time.time()
    running_time = end - start
    print('time cost : %.5f sec' % running_time)

    o3d.visualization.draw_geometries([scene_point_cloud, cropped_pcd1, cropped_pcd2, axes, bbox2, bbox1],
                                      mesh_show_back_face=False)
    if num_contact_points > coincide_num_points:
        return True, grasp_pose_euler, up_pose_euler
    else:
        return False, grasp_pose_euler, up_pose_euler


if __name__ == '__main__':
    matched_matrix = np.array([[9.73662869e-01  ,2.27042627e-01 ,- 2.07909325e-02 ,- 7.33713146e+01],
     [-9.88693008e-02 , 5.02641687e-01, 8.58822564e-01,- 2.32358699e+01],
    [2.05439720e-01 ,- 8.34148056e-01,5.11851093e-01,8.58239319e+02],
    [0.00000000e+00 , 0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]])
    matched_item = 't_pipe_u'


    coincide_num_points = 80
    is_collided = contact_detect(matched_item, matched_matrix, coincide_num_points)

    print(is_collided)
    if is_collided:
        print("点云重合，发生碰撞")
    else:
        print("正常执行抓取")




