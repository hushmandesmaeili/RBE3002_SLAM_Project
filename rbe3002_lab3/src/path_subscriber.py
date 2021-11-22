#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue

class PathSubscriber:

    def __init__(self):
        
        rospy.init_node("path_subscriber")

def path_subscriber(self, msg):
    rospy.loginfo("Requesting the path")
        rospy.wait_for_service('plan_path')
        pathdata = rospy.ServiceProxy('plan_path', GetPlan)
        try:
            resp1 = pathdata()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        # print(resp1.map)
        return resp1.plan