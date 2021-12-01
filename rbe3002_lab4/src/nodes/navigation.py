#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue



class Navigation:

    
    def __init__(self):
        """
        Class constructor
        """
        
        rospy.sleep(1.0)
        rospy.loginfo("Navigation node ready")



   
    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        rospy.spin()


        
if __name__ == '__main__':
    Navigation().run()
