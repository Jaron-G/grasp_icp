#!/usr/bin/env python3
import rospy
import argparse
import tkinter as tk
from tkinter import ttk

from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

class GripperControl:
    def __init__(self):
        """Initialize the application with a slider to control the gripper."""
        # Initialize ROS node
        # rospy.init_node('robotiq_gripper_controller', anonymous=True)
        
        # Initialize ROS publisher
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

        # Wait for the gripper to be ready
        rospy.sleep(1)
        # Send reset and activate command
        self.reset_gripper()
        self.activate_gripper()
    
    def reset_gripper(self):
        """Reset the gripper."""
        reset_command = outputMsg.Robotiq2FGripper_robot_output(rACT=0)
        self.pub.publish(reset_command)
        rospy.sleep(1)  # Ensure the command has time to be processed

    def activate_gripper(self):
        """Activate the gripper."""
        activate_command = outputMsg.Robotiq2FGripper_robot_output(rACT=1, rGTO=1, rSP=255, rFR=150)
        self.pub.publish(activate_command)
        rospy.sleep(1)  # Ensure the command has time to be processed

    def control_gripper(self, open_witdth):
        """Control the gripper position.
        @open_witdth: the width of the gripper opening in the range [0, 255].
        """
        command = outputMsg.Robotiq2FGripper_robot_output(rACT=1, rGTO=1, rSP=255, rFR=150, rPR=int(float(open_witdth)))
        self.pub.publish(command)

def main():
    # Initialize the gripper control application
    gripper = GripperControl()

    # Demo: create slider to control the gripper
    master = tk.Tk()
    master.title("Robotiq Gripper Control")
    slider = ttk.Scale(master, from_=0, to=255, orient="horizontal", command=gripper.control_gripper, length=300)
    slider.pack(fill=tk.X, expand=True)
    master.mainloop()

if __name__ == '__main__':
    main()