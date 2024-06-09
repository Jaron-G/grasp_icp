#!/usr/bin/env python3
import rospy
import tkinter as tk

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
        print("Service call failed: %s" % e)

def registration_client():
    rospy.wait_for_service('registration')
    try:
        registration = rospy.ServiceProxy('registration', RegistratePose)
        resp1 = registration()
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

    def create_widgets(self):
        self.frame1 = tk.Frame(self)
        self.frame2 = tk.Frame(self)
        self.obtain_pcd_button = tk.Button(self.frame1, text="Get scene point cloud", command=self.press_obtain_pcd_button,width=28
                                           # , height=2,font=32
                                           )
        self.obtain_pcd_button.pack(pady=5)
        self.registration_button = tk.Button(self.frame1, text="Perform registration", command=self.press_registration_button,width=28)
        self.registration_button.pack(pady=5)
        self.pose_transformation_button = tk.Button(self.frame1, text="Perform pose transformation", command=self.press_pose_transformation_button,width=28)
        self.pose_transformation_button.pack(pady=5)
        self.move_robot_button = tk.Button(self.frame1, text="Start grasp", command=self.press_move_robot_button,width=28)
        self.move_robot_button.pack(pady=5)
        self.quit = tk.Button(self.frame1, text="QUIT", fg="red", command=self.master.destroy, bd=3,width=28)
        self.quit.pack(pady=5)

        self.photo = tk.PhotoImage(file="/catkin_ws/src/grasp_icp/src/main/robot.png")
        self.photo_label = tk.Label(self.frame2,justify = tk.LEFT,image = self.photo)
        self.photo_label.pack(side="right")

        self.message_var = tk.StringVar(self,value='Please click the buttons in order !')  # 储存文字的类
        self.message_box = tk.Label(self.frame1, textvariable=self.message_var, bg='lightblue',width=28)
        self.message_box.pack(pady=5)

        self.frame1.pack(side="left")  # 左框架对齐
        self.frame2.pack(side="right")  # 右框架对齐

    def press_obtain_pcd_button(self):
        print("Start obtain point cloud !")
        rospy.loginfo("Start obtain point cloud !")
        response = obtain_pcd_client()
        rospy.loginfo("Obtain point cloud completed!")
        self.message_var.set("Obtain point cloud completed!")

    def press_registration_button(self):
        print("Start perform registration !")
        rospy.loginfo("Start perform registration !")
        response = registration_client()
        self.final_pose, self.matched_model = response.final_pose, response.matched_model
        rospy.loginfo("Registration completed !")
        self.message_var.set("Registration completed !")

    def press_pose_transformation_button(self):
        print("Pose transformation !")
        rospy.loginfo("Pose transformation !")
        response = pose_transformation_client(self.final_pose, self.matched_model)
        self.rotation_matrix, self.pose_g, self.pose_up = response.rotation_matrix, response.grasp_pose, response.up_pose
        rospy.loginfo("Pose transformation completed !")
        self.message_var.set("Pose transformation completed !")

    def press_move_robot_button(self):
        print("Start grasp !")
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
    root.geometry('400x240')
    app = Application(master=root)
    app.mainloop()

if __name__ == '__main__':
    rospy.init_node('main_process')
    main()
    rospy.spin()
