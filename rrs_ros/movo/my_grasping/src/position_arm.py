#!/usr/bin/env python

import sys, rospy
from std_msgs.msg import Bool
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from movo_action_clients.gripper_action_client import GripperActionClient
import moveit_commander

if __name__ == "__main__":
	rospy.init_node('position_arm')
	
	move_group = MoveGroupInterface("right_arm","base_link")
	#move_group.setPlannerId("RRTConnectkConfigDefault")
	lgripper = GripperActionClient('left')
	rgripper = GripperActionClient('right')
	#dof = rospy.get_param('~jaco_dof')
	rarm_joints = ["right_shoulder_pan_joint","right_shoulder_lift_joint","right_arm_half_joint","right_elbow_joint","right_wrist_spherical_1_joint","right_wrist_spherical_2_joint","right_wrist_3_joint"]
	newpos = [0,0.07,0.0175,1.448,0.01745,0.0349,1.4835]
	grasp = [-3.0,  2.0, 0.0,  2.0, 0.0, 0.0,  1.4]#, 2.6, -2.0, 0.0, -2.0, 0.0, 0.0, -1.0, 0.371, 0,0]
	tucked = [-1.6,-1.4,0.4,-2.7,0.0,0.5,-1.7]#, 1.6,1.4,-0.4,2.7,0.0,-0.5, 1.7, 0.04, 0, 0]
	success=False
	while not rospy.is_shutdown() and not success:
		
		result = move_group.moveToJointPosition(rarm_joints, grasp, 0.05)
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			success = True
		else:
			rospy.logerr("moveToJointPosition failed (%d)" %result.error_code.val)
	
	rospy.sleep(5)
	while not rospy.is_shutdown() and not success:
		result = move_group.moveToJointPosition(upper_body_joints, tucked, 0.05)
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			success = True


	rospy.sleep(5)
