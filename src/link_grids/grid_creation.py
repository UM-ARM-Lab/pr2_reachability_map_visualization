links = ["rHumerus", "rRadius", "rHand"]
for link in links:
    temp = l.active.GetLink(link)
    temp.SetTransform(np.eye(4))
    aabb = temp.ComputeAABB()
    ppoints = []
    stepsize = 0.01
    for x in np.arange(aabb.pos()[0]-aabb.extents()[0], aabb.pos()[0]+aabb.extents()[0], stepsize):
        for y in np.arange(aabb.pos()[1]-aabb.extents()[1], aabb.pos()[1]+aabb.extents()[1], stepsize):
            for z in np.arange(aabb.pos()[2]-aabb.extents()[2], aabb.pos()[2]+aabb.extents()[2], stepsize):
                ppoints.append([x,y,z])

    np.savetxt(link+".csv", np.array(ppoints), delimiter=",")


    test = l.orEnv.plot3(points=np.array(ppoints), 
                                   pointsize= 0.01, 
                                   colors=array(((0,0,1))), 
                                   drawstyle=0)