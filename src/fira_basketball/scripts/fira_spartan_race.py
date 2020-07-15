#! /usr/bin/python

import rospy
import numpy as np
import cv2

from op3_ros_utils import getWalkingParams, Robot
from vision import *
from copy import copy
import sys

DEBUG_MODE = False # Show the detected image
MIN_AREA = 1500

class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_TO_EDGE = 1 # Moves the head, looking for the ball
    CLIMB = 2 # Moves the head, looking for the ball
    DESCEND = 3
    END = 99

# Functions to be passed to vision system
#func1 = detectSingleColor
##func1 = detect2Color
#args1 = ((np.array([13, 150, 100]), np.array([28, 255, 255])),)
#args1 = ((np.array([0, 120, 45]), np.array([10, 255, 255])),
#         (np.array([170, 120, 45]), np.array([180, 255, 255])))

# Create vision system
#vision = VisionSystem(pipeline_funcs=[func1],
#                      pipeline_args=[args1], debug=DEBUG_MODE, verbose=1)

# Subscribe to cv_camera topic with vision system
#rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)

# Create robot
robot = Robot()

# Iinitialize Node
rospy.init_node("fira_spartan_race")

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")

    robot.setGrippersPos(left=0.0, right=0.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-0.8])

    rospy.sleep(1.0)

def center_head_on_object(obj_pos):
    cx, cy = 0.5, 0.5 # Center of image
    obj_x, obj_y = obj_pos

    dist_x = obj_x - cx
    dist_y = obj_y - cy

    head_curr_x = robot.joint_pos["head_pan"]
    head_curr_y = robot.joint_pos["head_tilt"]

    kp = 0.5
    new_head_x = head_curr_x + kp * -dist_x
    new_head_y = head_curr_y + kp * -dist_y

    robot.setJointPos(["head_tilt", "head_pan"], [new_head_y, new_head_x])

tickrate = 60
rate = rospy.Rate(tickrate)

# TODO remember to put to 0
STEP_LEVEL = 0

currState = States.INIT
while not rospy.is_shutdown():
    if robot.buttonCheck("mode"):
        STEP_LEVEL = 0
        currState = States.INIT

    if DEBUG_MODE:
        if vision.status[0]:
            cv2.imshow("Frame", vision.debug_img[0])
            cv2.waitKey(1)

    if currState == States.INIT:
        print("[INIT]")
        init()

        # Transition
        tick_count = 0
        direction = False
        currState = States.READY

    elif currState == States.READY:
        print("[READY]")
        #if robot.buttonCheck("start"):
        robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        tick_count = 0
        robot.walkStart()
        currState = States.WALK_TO_EDGE
    
    elif currState == States.WALK_TO_EDGE:
        print("[WALK_TO_EDGE]")
        #if vision.status[0] and vision.results[0][1] > MIN_AREA:
        #    center, area = vision.results[0]
        #    center_head_on_object(center)

        pan_angle = robot.joint_pos["head_pan"]

        robot.walkVelocities(x=6.5, th=0.0, z_move_amplitude=0.017, balance=True,
                z_offset=0.065, hip_pitch=8)

        tick_count += 1

        # TODO change this
        if tick_count > 12 * tickrate:
            tick_count = 0
            currState = States.CLIMB

    elif currState == States.CLIMB:
        print("[CLIMB]")
        print("Step Level: {}".format(STEP_LEVEL))

        STEP_LEVEL += 1

        robot.setGeneralControlModule("action_module")
        robot.playMotion(52, wait_for_end=True)
        robot.playMotion(79, wait_for_end=True)
        robot.setGeneralControlModule("walking_module")
        robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        robot.setJointPos(["head_tilt"], [-0.8])
        robot.walkStart()
        
        if STEP_LEVEL < 3:
            currState = States.WALK_TO_EDGE
        else:
            robot.setGeneralControlModule("action_module")
            rospy.sleep(5)
            robot.playMotion(40, wait_for_end=True)
            currState = States.DESCEND

    elif currState == States.DESCEND:
        #robot.setGeneralControlModule("action_module")
        robot.playMotion(63, wait_for_end=True)

    elif currState == States.END:
        print("[END]")
        robot.walkStop()

    rate.sleep()
