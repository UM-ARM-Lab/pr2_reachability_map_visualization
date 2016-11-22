#!/usr/bin/env python
import time
import openravepy
import numpy as np

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('robots/pr2-beta-static.zae')
    # env.Load('/home/rafi/Dropbox/catkin_ws/src/hrics-or-rafi/ormodels/human_wpi_bio.xml')
    time.sleep(0.1)
    time.sleep(2)

    prob = RaveCreateProblem(env,'bridge')
    env.LoadProblem(prob, "")

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    res = 0.02

    prob.SendCommand("GetLinkSamples /home/rafi/Dropbox/catkin_ws/src/or_bridge/src/link_grids/pr2_large/ pr2 "+str(res)+ " r_elbow_flex_link r_forearm_link r_forearm_roll_link r_gripper_l_finger_link r_gripper_l_finger_tip_link r_gripper_palm_link r_gripper_r_finger_link r_gripper_r_finger_tip_link r_upper_arm_link r_wrist_flex_link r_wrist_roll_link")
    # prob.SendCommand("GetLinkSamples /home/rafi/Dropbox/catkin_ws/src/or_bridge/src/link_grids/human_large/ " + robot.GetName() + " " + str(res)+ " rHand rHumerus rRadius")
    robot.SetVisible(False)


    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

