#include "ros/ros.h"
#include <stdexcept>
#include <tr1cpp/tr1.h>
#include <tr1cpp/segment.h>
#include <tr1cpp/joint.h>

namespace tr1cpp
{
	TR1::TR1()
	{
		for (int i = 0; i < 8; i++)
		{
			armRight.joints[i].setMotorId(i + 1);
			armLeft.joints[i].setMotorId(i + 16);
		}

		//base
		base.joints[0].name = "JointBaseWheelFL";
		base.joints[0].setMotorId(9);
		base.joints[1].name = "JointBaseWheelFR";
		base.joints[1].setMotorId(10);
		base.joints[2].name = "JointBaseWheelBL";
		base.joints[2].setMotorId(11);
		base.joints[3].name = "JointBaseWheelBR";
		base.joints[3].setMotorId(12);
		base.joints[4].name = "JointTorsoExtension";
		base.joints[4].setMotorId(13);

		//head
		head.joints[0].name = "neck_base_to_neck";
		head.joints[0].setMotorId(14);
		head.joints[0].sensorResolution = 128;
		head.joints[1].name = "neck_to_head";
		head.joints[1].setMotorId(15);
		head.joints[1].sensorResolution = 128;

		//torso
		torso.joints[0].name = "JointTorsoExtension";
		torso.joints[0].setMotorId(13);

		//armRight
		armRight.joints[0].name = "JointRightShoulderPan";
		armRight.joints[0].setMotorId(1);
		armRight.joints[1].name = "JointRightShoulderTilt";
		armRight.joints[1].setMotorId(2);
		armRight.joints[2].name = "JointRightUpperArmRoll";
		armRight.joints[2].setMotorId(3);
		armRight.joints[3].name = "JointRightElbowFlex";
		armRight.joints[3].setMotorId(4);
		armRight.joints[4].name = "JointRightForearmRoll";
		armRight.joints[4].setMotorId(5);
		armRight.joints[5].name = "JointRightWristFlex";
		armRight.joints[5].setMotorId(6);
		armRight.joints[6].name = "JointRightWristRoll";
		armRight.joints[6].setMotorId(7);
		armRight.joints[7].name = "JointRightGripper";
		armRight.joints[7].setMotorId(8);

		armRight.joints[6].setActuatorType(ACTUATOR_TYPE_SERVO);
		armRight.joints[7].setActuatorType(ACTUATOR_TYPE_SERVO);

		armRight.joints[6].setServoLimits(0, 180);
		armRight.joints[7].setServoLimits(0, 150);

		armRight.joints[6].actuate(0, 15);
		ROS_INFO("wrist set to 0");

		//armLeft
		armLeft.joints[0].name = "JointLeftShoulderPan";
		armLeft.joints[0].setMotorId(16);
		armLeft.joints[1].name = "JointLeftShoulderTilt";
		armLeft.joints[1].setMotorId(17);
		armLeft.joints[2].name = "JointLeftUpperArmRoll";
		armLeft.joints[2].setMotorId(18);
		armLeft.joints[3].name = "JointLeftElbowFlex";
		armLeft.joints[3].setMotorId(19);
		armLeft.joints[4].name = "JointLeftForearmRoll";
		armLeft.joints[4].setMotorId(20);
		armLeft.joints[5].name = "JointLeftWristFlex";
		armLeft.joints[5].setMotorId(21);
		armLeft.joints[6].name = "JointLeftWristRoll";
		armLeft.joints[6].setMotorId(22);
		armLeft.joints[7].name = "JointLeftGripper";
		armLeft.joints[7].setMotorId(23);

		armLeft.joints[6].setActuatorType(ACTUATOR_TYPE_SERVO);
		armLeft.joints[7].setActuatorType(ACTUATOR_TYPE_SERVO);

		armLeft.joints[6].setServoLimits(0, 180);
		armLeft.joints[7].setServoLimits(20, 120);
	}

	TR1::~TR1()
	{

	}

	Joint TR1::getJoint(std::string jointName)
	{
		int numJointsHead = sizeof(head.joints) / sizeof(head.joints[0]);
		for (int i = 0; i < numJointsHead; i++)
		{
			if (head.joints[i].name == jointName)
			{
				return head.joints[i];
			}
		}

		int numJointsBase = sizeof(base.joints) / sizeof(base.joints[0]);
		for (int i = 0; i < numJointsBase; i++)
		{
			if (base.joints[i].name == jointName)
			{
				return base.joints[i];
			}
		}

		int numJointsTorso = sizeof(torso.joints) / sizeof(torso.joints[0]);
		for (int i = 0; i < numJointsTorso; i++)
		{
			if (torso.joints[i].name == jointName)
			{
				return torso.joints[i];
			}
		}

		int numJointsRight = sizeof(armRight.joints) / sizeof(armRight.joints[0]);
		for (int i = 0; i < numJointsRight; i++)
		{
			if (armRight.joints[i].name == jointName)
			{
				return armRight.joints[i];
			}
		}

		int numJointsLeft = sizeof(armLeft.joints) / sizeof(armLeft.joints[0]);
		for (int i = 0; i < numJointsLeft; i++)
		{
			if (armLeft.joints[i].name == jointName)
			{
				return armLeft.joints[i];
			}
		}

		throw std::runtime_error("Could not find joint with name " + jointName);
	}

	void TR1::setJoint(Joint joint)
	{
		bool foundJoint = false;

		int numJointsHead = sizeof(head.joints) / sizeof(head.joints[0]);
		for (int i = 0; i < numJointsHead; i++)
		{
			if (head.joints[i].name == joint.name)
			{
				foundJoint = true;
				head.joints[i] = joint;
			}
		}

		int numJointsBase = sizeof(base.joints) / sizeof(base.joints[0]);
		for (int i = 0; i < numJointsBase; i++)
		{
			if (base.joints[i].name == joint.name)
			{
				foundJoint = true;
				base.joints[i] = joint;
			}
		}

		int numJointsTorso = sizeof(torso.joints) / sizeof(torso.joints[0]);
		for (int i = 0; i < numJointsTorso; i++)
		{
			if (torso.joints[i].name == joint.name)
			{
				foundJoint = true;
				torso.joints[i] = joint;
			}
		}

		int numJointsRight = sizeof(armRight.joints) / sizeof(armRight.joints[0]);
		for (int i = 0; i < numJointsRight; i++)
		{
			if (armRight.joints[i].name == joint.name)
			{
				foundJoint = true;
				armRight.joints[i] = joint;
			}
		}

		int numJointsLeft = sizeof(armLeft.joints) / sizeof(armLeft.joints[0]);
		for (int i = 0; i < numJointsLeft; i++)
		{
			if (armLeft.joints[i].name == joint.name)
			{
				foundJoint = true;
				armLeft.joints[i] = joint;
			}
		}

		if (foundJoint == false)
		{
			throw std::runtime_error("Could not find joint with name " + joint.name);
		}
	}
}
