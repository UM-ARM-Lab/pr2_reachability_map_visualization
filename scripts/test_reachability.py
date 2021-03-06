import numpy as np
from openravepy import *


if __name__ == "__main__":
    env = Environment()
    env.SetViewer("qtcoin")
    env.GetViewer().SetCamera(np.array([[-0.99864759, -0.03733027, -0.03618612,  1.77346683],
                                   [ 0.01592122,  0.44299137, -0.8963845 ,  3.07433772],
                                   [ 0.04949241, -0.89574835, -0.44179792,  1.91713417],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]]))
    env.Load('robots/pr2-beta-static.zae')
    robot = env.GetRobot('pr2')
    prob = RaveCreateProblem(env, 'reachabilitymap')
    # prob.SendCommand("InitReachabilityMap"+" " +"pr2"+" "+"rightarm" + " " + "/home/ruikun/Dropbox/ARC_workspace/src/reachability_map/scripts/pr2_rightarm_1cm.rmg")
    prob.SendCommand("InitReachabilityMap"+" " +"pr2"+" "+"rightarm" + " " + "/home/ruikun/Dropbox/ARC_workspace/src/reachability_map/scripts/pr2_right_arm_5cm_col_nolock.rmg")
    prob.SendCommand("DrawGrid")
    # prob.SendCommand("SendCommand")


    cost = prob.SendCommand("GetReachabilityMapCost"+" "+"0.8"+" "+"0"+" "+"0")
    print "old cost: ", float(cost)

    prob.SendCommand("UpdateMap" +" "+"0.8"+" "+"0"+" "+"0" + " "+"0")
    cost = prob.SendCommand("GetReachabilityMapCost"+" "+"0.8"+" "+"0"+" "+"0")
    print "new cost: ", float(cost)

    # cbirrt = CBiRRT(env,'pr2')
    # cbirrt.DoGeneralIK(execute = False, gettime = False, norot = False, returncloset = False, )

    robot.WaitForController(0)
    raw_input('finish')
