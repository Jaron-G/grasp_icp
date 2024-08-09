#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import tkinter as tk
import numpy as np

# from obtain_pcd.srv import ObtainPcd
from halcon_package.srv import RegistratePose
from pose_transformation.srv import PoseTransform
from move_robot.srv import MoveToPose

DEBUG = False

from reconstruction.srv import Reconstruction, ReconstructionRequest
from projector.srv import Projection, ProjectionRequest
from basler_camera.srv import Reset_parameters, Reset_parametersRequest

from cropping_method import contact_detect

from real_gripper_control import GripperControl


def request_projection():
    # 等待投影服务
    rospy.wait_for_service('make_projections')
    # 等待相机参数重置服务
    rospy.wait_for_service('reset_parameters')
    try:
        projection_service = rospy.ServiceProxy('make_projections', Projection)
        request = ProjectionRequest()
        response = projection_service(request)
        if response.success:
            # 如果投影成功，调用相机参数重置服务
            rospy.loginfo("Projection successful")
            try:
                reset_parameters = rospy.ServiceProxy('reset_parameters', Reset_parameters)
                request = Reset_parametersRequest()
                response = reset_parameters(request)
                if response.success:
                    rospy.loginfo("Successfully reset parameters")
                else:
                    rospy.loginfo("Failed to reset parameters")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
        else:
            rospy.loginfo("Projection failed")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def request_reconstruction():
    rospy.wait_for_service('make_reconstruction')
    try:
        reconstruction_service = rospy.ServiceProxy('make_reconstruction', Reconstruction)
        request = ReconstructionRequest()
        response = reconstruction_service(request)
        if response.success:
            rospy.loginfo("Reconstruction successful")
        else:
            rospy.loginfo("Reconstruction failed")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def registration_client(debug_mode):
    rospy.wait_for_service('registration')
    try:
        registration = rospy.ServiceProxy('registration', RegistratePose)
        resp1 = registration(debug_mode)
        return resp1
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def pose_transformation_client(x, y):
    rospy.wait_for_service('pose_transformation')
    try:
        pose_transformation = rospy.ServiceProxy('pose_transformation', PoseTransform)
        resp1 = pose_transformation(x, y)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def move_robot_to_pose_client(x, y, z):
    rospy.wait_for_service('move_to_pose_service')
    try:
        move_to_pose = rospy.ServiceProxy('move_to_pose_service', MoveToPose)
        resp1 = move_to_pose(x, y, z)
        return resp1.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

class Application(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()
        self.matched_model = None
        self.final_pose = None
        self.pose_up = None
        self.pose_g = None
        self.rotation_matrix = None
        self.pose_g_with_offset = None

    def create_widgets(self):
        self.frame1 = tk.Frame(self)
        self.frame2 = tk.Frame(self)
        self.reset_vrobot_button = tk.Button(self.frame1, text="projection_button", command=self.press_projection_button, width=28)
        self.reset_vrobot_button.pack(pady=5)
        self.obtain_pcd_button = tk.Button(self.frame1, text="reconstruction_button", command=self.press_reconstruction_button, width=28)
        self.obtain_pcd_button.pack(pady=5)
        self.registration_button = tk.Button(self.frame1, text="Perform registration", command=self.press_registration_button, width=28)
        self.registration_button.pack(pady=5)
        self.pose_transformation_button = tk.Button(self.frame1, text="Perform pose transformation", command=self.press_pose_transformation_button, width=28)
        self.pose_transformation_button.pack(pady=5)
        self.collision_detect_button = tk.Button(self.frame1, text="Perform collision detect", command=self.press_collision_detect_button, width=28)
        self.collision_detect_button.pack(pady=5)
        self.move_robot_button = tk.Button(self.frame1, text="Start grasp", command=self.press_move_robot_button, width=28)
        self.move_robot_button.pack(pady=5)
        self.quit = tk.Button(self.frame1, text="QUIT", fg="red", command=self.master.destroy, bd=3, width=28)
        self.quit.pack(pady=5)

        self.photo = tk.PhotoImage(file="/catkin_ws/src/grasp_icp/src/main/robot.png")
        self.photo_label = tk.Label(self.frame2,justify = tk.LEFT,image = self.photo)
        self.photo_label.pack(side="right")

        self.message_var = tk.StringVar(self,value='Please click the buttons in order !')  # 储存文字的类
        self.message_box = tk.Label(self.frame1, textvariable=self.message_var, bg='lightblue',width=28)
        self.message_box.pack(pady=5)

        self.debug_var = tk.IntVar()
        self.debug_checkbox = tk.Checkbutton(self, text="Debug Mode", variable=self.debug_var, command=self.debug_check)
        self.debug_checkbox.pack()

        self.frame1.pack(side="left")  # 左框架对齐
        self.frame2.pack(side="right")  # 右框架对齐

    # 在终端打印当前模式信息
    def debug_check(self):
        if  self.debug_var.get()==1 :
            print("Start debug mode !") 
        else:
            print("Start normal mode !")

    def press_projection_button(self):
        rospy.loginfo("Start projection !")
        response = request_projection()
        rospy.loginfo("projection completed!")
        self.message_var.set("projection completed!")

    def press_reconstruction_button(self):
        rospy.loginfo("reconstruction !")
        request_reconstruction()
        rospy.loginfo("reconstruction successfully!")
        self.message_var.set("reconstruction successfully !")

    def press_registration_button(self):
        rospy.loginfo("Start perform registration !")
        if self.debug_var.get() == 1:
            debug_mode = True
        else:
            debug_mode = False
        response = registration_client(debug_mode)
        self.final_pose, self.matched_model = response.final_pose, response.matched_model
        rospy.loginfo("Registration completed !")
        self.message_var.set("Registration completed !")

    def press_pose_transformation_button(self):
        rospy.loginfo("Pose transformation !")
        response = pose_transformation_client(self.final_pose, self.matched_model)
        self.rotation_matrix, self.pose_g, self.pose_up = response.rotation_matrix, response.grasp_pose, response.up_pose
        print(self.pose_g)


        
        print(self.final_pose)
    
        # gripper = GripperControl()
        # gripper.control_gripper(0)
        rospy.loginfo("Pose transformation completed !")
        self.message_var.set("Pose transformation completed !")

    def press_collision_detect_button(self):
        matched_matrix = np.array(self.final_pose).reshape(4,4)
        coincide_num_points = 80
        is_collided,pose_g,pose_up= contact_detect(self.matched_model,matched_matrix,coincide_num_points)
        offset = 0
        while is_collided:
            print("点云重合，发生碰撞,建以重新规划！")
            offset = offset + 70
            is_collided,pose_g,pose_up = contact_detect(self.matched_model, matched_matrix, coincide_num_points, offset)
            print(is_collided)
        print("正常执行抓取。")
        
        # self.pose_g_with_offset = tuple(pose_g)
        # self.pose_up_with_offset = tuple(pose_up)
        self.pose_g = tuple(pose_g)
        self.pose_up = tuple(pose_up)

        # print("pose_g_with_offset:",self.pose_g_with_offset)
        # print("pose_g_up_with_offset:",self.pose_up_with_offset)
        print("pose_g:",self.pose_g)
        print("pose_g_up:",self.pose_up)

    def press_move_robot_button(self):
        rospy.loginfo("Start grasp !")
        result = move_robot_to_pose_client(self.rotation_matrix, self.pose_g, self.pose_up)
        if result:
            rospy.loginfo("Robot moved successfully!")
        else:
            rospy.loginfo("Failed to move robot.")
        self.message_var.set("Robot moved successfully!")
    

def main():
    root = tk.Tk()
    root.title('Pipe grasping based on ICP registration')
    root.geometry('400x320')
    app = Application(master=root)
    app.mainloop()

if __name__ == '__main__':
    rospy.init_node('main_process')
    main()
    rospy.spin()
