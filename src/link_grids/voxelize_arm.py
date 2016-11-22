#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
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

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('robots/pr2-beta-static.zae')
    # env.Load('/home/rafi/workspace/hrics-or-rafi/ormodels/human_wpi_bio.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    res = 0.01

    # tuck in the PR2's arms for driving
    # tuckarms(env,robot);

    with env:
        colbox = RaveCreateKinBody(env,'')
        colbox.SetName('colbox')
        colbox.InitFromBoxes(numpy.array([[0,0,0,res/2.0,res/2.0,res/2.0]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
        env.Add(colbox,True)

    handles = []
    all_points = []
    #### YOUR CODE HERE ####
    with env:
        #Human
        # links = ["rHand", "rHumerus", "rRadius"]
        # for p in links:
        #     link = robot.GetLink(p)

        # for link in robot.GetLinks():
        #     if link.GetName().startswith("r_"):
        links = links = "r_elbow_flex_link r_forearm_link r_forearm_roll_link r_gripper_l_finger_link r_gripper_l_finger_tip_link r_gripper_palm_link r_gripper_r_finger_link r_gripper_r_finger_tip_link r_upper_arm_link r_wrist_flex_link r_wrist_roll_link".split()
        for link_name in links:
            if True:
                link = robot.GetLink(link_name)
                # link.SetTransform(np.eye(4))
                aabb = link.ComputeAABB()
                ppoints = []
                stepsize = 0.025
                for x in np.arange(aabb.pos()[0]-aabb.extents()[0], aabb.pos()[0]+aabb.extents()[0], stepsize):
                    for y in np.arange(aabb.pos()[1]-aabb.extents()[1], aabb.pos()[1]+aabb.extents()[1], stepsize):
                        for z in np.arange(aabb.pos()[2]-aabb.extents()[2], aabb.pos()[2]+aabb.extents()[2], stepsize):
                            t_box = np.eye(4)
                            t_box[0,3] = x
                            t_box[1,3] = y
                            t_box[2,3] = z
                            colbox.SetTransform(t_box)
                            if env.CheckCollision(colbox, link):
                                ppoints.append([x,y,z])

                if len(ppoints) > 0:
                    # np.savetxt(link.GetName()+".csv", np.array(ppoints), delimiter=",")
                    handles.append(env.plot3(points=np.array(ppoints), 
                                                   pointsize= 1.0, 
                                                   colors=array(((0,0,1))), 
                                                   drawstyle=0))
                    all_points.extend(ppoints)


    print "Tot # points? : ", len(all_points)
    robot.SetVisible(False)
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

