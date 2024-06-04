# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import moveit_commander

class gripper_2F85(object):
    """UR_robot moveit control class"""

    def __init__(self):
        super(gripper_2F85, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "gripper"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        print(self.move_group.get_current_state())
        self.move_group.set_max_velocity_scaling_factor(0.4)
        self.move_group.set_goal_joint_tolerance(0.001)
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.001)

    def open(self):
        move_group = self.move_group
        move_group.set_named_target('open')
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()      
        
    # def close(self):
    #     move_group = self.move_group
    #     move_group.set_named_target('close')
    #     success = move_group.go(wait=True)
    #     move_group.stop()
    #     move_group.clear_pose_targets()

    def close(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.455
        joint_goal[1] = -0.455
        joint_goal[2] = 0.455
        joint_goal[3] = 0.455
        joint_goal[4] = -0.455
        joint_goal[5] = 0.455
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    
