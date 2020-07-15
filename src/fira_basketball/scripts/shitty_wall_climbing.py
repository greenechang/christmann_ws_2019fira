#! /usr/bin/env python

import rospy
from op3_ros_utils import *

rospy.init_node("fahfghai")

robot = Robot()

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")

    robot.setGrippersPos(left=60.0, right=60.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-0.8])

    rospy.sleep(1.0)


init()

raw_input()
rospy.sleep(4)

robot.walkVelocities(x=3.5, th=0, z_move_amplitude=0.045, balance=True,
    z_offset=0.035)
robot.walkStart()
rospy.sleep(5)
robot.walkStop()

robot.setGeneralControlModule("action_module")
robot.playMotion(1, wait_for_end=True)
rospy.sleep(0.25)
robot.playMotion(58, wait_for_end=True)
robot.setGrippersPos(left=30.0, right=30.0)
rospy.sleep(0.25)
robot.playMotion(18, wait_for_end=True)

rospy.spin()


