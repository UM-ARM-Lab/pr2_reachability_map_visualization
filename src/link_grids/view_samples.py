#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy
import numpy as np
from os import listdir

from scipy.spatial import distance


if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def interp(path, dist_func, dt=0.05):
    new_path = []
    q_curr = path[0]
    new_path.append(q_curr)
    for q_next in path[1:]:
        dist = dist_func(q_next, q_curr)
        num_steps = np.ceil(np.linalg.norm(dist)/dt)
        if num_steps == 0:
            q_curr = q_next
            continue
        # num_steps = np.ceil(abs(dist.sum()/dt))

        ratio = np.arange(1,num_steps+1,1) / num_steps # Inclusive
        q_new = q_curr+np.array([dist*r for r in ratio])
        new_path.extend(q_new.tolist())
        q_curr = q_next
    return new_path

def do_camera_shit(start, end):
    xyz_start = start[:3,3]
    xyz_end = end[:3,3]

    print interp([xyz_start, xyz_end], distance.euclidean)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('robots/pr2-beta-static.zae')
    # env.Load('/home/rafi/Dropbox/catkin_ws/src/hrics-or-rafi/ormodels/human_wpi_bio.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    res = 0.0175
    base_dir = "/home/rafi/Dropbox/catkin_ws/src/or_bridge/src/link_grids/pr2_large_backup/"
    # base_dir = "/home/rafi/Dropbox/catkin_ws/src/or_bridge/src/link_grids/Human/"

    raw_input()

    with env:
        colbox = RaveCreateKinBody(env,'')
        colbox.SetName('colbox')
        colbox.InitFromBoxes(numpy.array([[0,0,0,res/2.0,res/2.0,res/2.0]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
        env.Add(colbox,True)

    handles = []
    all_points = []
    #### YOUR CODE HERE ####
    with env:
        for filename in listdir(base_dir):
            print filename
            if filename == "r_shoulder_pan_link.csv":
                continue
            # if filename == "r_upper_arm_link.csv":
            #     continue
            # if filename == "r_elbow_flex_link.csv":
            #     continue
            link = robot.GetLink(filename[:-4])

            ppoints = np.genfromtxt(base_dir+filename, delimiter=",")[:,:-1]
            T_link = link.GetTransform()[:3,3:].T
            ppoints += T_link

            if len(ppoints) > 0:
                # np.savetxt(link.GetName()+".csv", np.array(ppoints), delimiter=",")
                handles.append(env.plot3(points=np.array(ppoints), 
                                               pointsize= 3.0, 
                                               colors=array(((0,0,1))), 
                                               drawstyle=0))
                all_points.extend(ppoints.tolist())

            for link in robot.GetLinks():
                if link.GetName() == "r_shoulder_pan_link":
                    continue
                if link.GetName().startswith("r_"):
                    link.SetVisible(False)

            # print filename
            # robot.SetVisible(False)
            # raw_input("wait")


    # Do camera interp


    print "done"
    print "Tot # points? : ", len(all_points), len(set(map(tuple,all_points)))
    while True:
        raw_input()
        for link in robot.GetLinks():
            if link.GetName() == "r_shoulder_pan_link":
                continue
            if link.GetName().startswith("r_"):
                link.SetVisible(False)
        raw_input()
        robot.SetVisible(True)
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

