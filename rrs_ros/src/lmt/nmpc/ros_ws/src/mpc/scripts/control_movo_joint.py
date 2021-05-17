#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

if __name__ == '__main__':
	rospy.init_node('move_robot_using_trajectory_msg')
	topic_name = "movo/left_arm_controller/command"
	left_pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
	topic_name = "movo/right_arm_controller/command"
	right_pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
	jointCmd = JointTrajectory()  
	point = JointTrajectoryPoint()
	jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
	point.time_from_start = rospy.Duration.from_sec(5.0)
	jointCmd.joint_names = ["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_arm_half_joint",\
	 "left_elbow_joint", "left_wrist_spherical_1_joint", "left_wrist_spherical_2_joint", "left_wrist_3_joint"]
	point.positions=[1.553,1.502,-0.412,2.6,-0.024,-0.478,1.703]
	jointCmd.points.append(point)

	jointCmd1 = JointTrajectory() 
	point1 = JointTrajectoryPoint()
	jointCmd1.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
	point1.time_from_start = rospy.Duration.from_sec(5.0)
	jointCmd1.joint_names = ["right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_arm_half_joint", "right_elbow_joint",\
                "right_wrist_spherical_1_joint", "right_wrist_spherical_2_joint", "right_wrist_3_joint"]
	point1.positions=[1.553,1.502,0.412,2.6,-0.024,-0.478,1.703]
	jointCmd1.points.append(point1)
	rate = rospy.Rate(100)
	count = 0
	while (count < 50):
		left_pub.publish(jointCmd)
		right_pub.publish(jointCmd1)
		count = count + 1
		rate.sleep()

