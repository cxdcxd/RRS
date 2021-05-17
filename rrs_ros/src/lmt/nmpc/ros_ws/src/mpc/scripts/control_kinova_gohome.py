#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

if __name__ == '__main__':
	rospy.init_node('kinova_go_home')
	topic_name = "/j2s7s300/position_joint_trajectory_controller/command"
	left_pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
	jointCmd = JointTrajectory()  
	point = JointTrajectoryPoint()
	jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
	point.time_from_start = rospy.Duration.from_sec(2.0)
	jointCmd.joint_names = ['j2s7s300_joint_1', 'j2s7s300_joint_2', 'j2s7s300_joint_3', 'j2s7s300_joint_4', 'j2s7s300_joint_5',
  							'j2s7s300_joint_6', 'j2s7s300_joint_7']
	point.positions=[0.0,2.9,0.0,1.3,-2.07,1.4,0.0]
	jointCmd.points.append(point)

	rate = rospy.Rate(100)
	count = 0
	while (count < 50):
		left_pub.publish(jointCmd)
		count = count + 1
		rate.sleep()

