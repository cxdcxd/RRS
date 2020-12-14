#!/usr/bin/env python

import sys, rospy
from std_msgs.msg import Bool
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from movo_action_clients.gripper_action_client import GripperActionClient
import moveit_commander

if __name__ == "__main__":
	rospy.init_node('goto_tucked')
	
	move_group = MoveGroupInterface("right_arm","base_link")
	#move_group.setPlannerId("RRTConnectkConfigDefault")
	lgripper = GripperActionClient('left')
	rgripper = GripperActionClient('right')
	#dof = rospy.get_param('~jaco_dof')
	rarm_joints = ["right_shoulder_pan_joint","right_shoulder_lift_joint","right_arm_half_joint","right_elbow_joint","right_wrist_spherical_1_joint","right_wrist_spherical_2_joint","right_wrist_3_joint"]
	
	tucked = [-1.6,-1.4,0.4,-2.7,0.0,0.5,-1.7]#, 1.6,1.4,-0.4,2.7,0.0,-0.5, 1.7, 0.04, 0, 0]
	success=False
	while not rospy.is_shutdown() and not success:
		
		result = move_group.moveToJointPosition(rarm_joints, tucked, 0.05)
		if result.error_code.val == MoveItErrorCodes.SUCCESS:
			success = True
		else:
			rospy.logerr("moveToJointPosition failed (%d)" %result.error_code.val)
			#move_group.moveToJointPosition(upper_body_joints, tucked, 0.05)

	rospy.sleep(5)
