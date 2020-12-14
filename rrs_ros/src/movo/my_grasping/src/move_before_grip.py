#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from movo_action_clients.gripper_action_client import GripperActionClient


from sensor_msgs.msg import JointState

def main():
	rospy.init_node('move_before_grip')
	arm_group = moveit_commander.MoveGroupCommander("right_arm")
	pose_target = arm_group.get_current_pose()
	pose_target.pose.position.y -= 0.2
	arm_group.set_pose_target(pose_target)
	plan1 = arm_group.go()
	rospy.sleep(5)

if __name__ == "__main__":
    main()
