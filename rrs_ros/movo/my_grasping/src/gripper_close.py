#!/usr/bin/env python

import sys
from copy import copy
import rospy
import actionlib
import math
import random

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

from sensor_msgs.msg import JointState


class GripperActionTest(object):
    def __init__(self,prefix="right"):
        
        self._prefix = prefix
        self._client = actionlib.SimpleActionClient(
            '/movo/%s_gripper_controller/gripper_cmd'%self._prefix,
            GripperCommandAction,
        )
        self._goal = GripperCommandGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Gripper Command"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def command(self, position, block=False, timeout=15.0):
        self._goal.command.position = position
        self._goal.command.max_effort = -1.0
        self._client.send_goal(self._goal)
        if block:
            self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()


def main():
    rospy.init_node('gripper_close')

    rg_test = GripperActionTest("right")
    lg_test = GripperActionTest("left")
    
    
    rg_test.command(0.050)
    rg_test.wait()
    
    
    print("Gripper Action Test Example Complete")
    
if __name__ == "__main__":
    main()
