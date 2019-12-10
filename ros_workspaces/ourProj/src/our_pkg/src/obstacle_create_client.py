#!/usr/bin/env python
import sys
import rospy
from our_pkg.srv import obstacle_create_srv

def obstacle_create_client():
    rospy.wait_for_service('create_obstacle')
    try:
        create_obstacle = rospy.ServiceProxy('create_obstacle', obstacle_create_srv)
        # resp = create_obstacle(pa[0],pa[1],pa[2],pa[3],pa[4],pa[5])
        resp = create_obstacle("obs3", 0.45, -0.65, 0.8, 0.8, 10)
        # resp = create_obstacle("obs2", -0.25, 0.25, 0.3, 0.3, 10)
        print("Done")
        return resp
    except:
        print("Service call failed")
    
if __name__ == "__main__":
    # if len(sys.argv) == 8:
    #     para = sys.argv[3:]
    #     para = [float(i) for i in para]
    #     para.insert(0, sys.argv[2])
    # else:
    #     sys.exit(1)
    a = obstacle_create_client()
