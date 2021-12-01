#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue



class FrontierExploration:

    
    def __init__(self):
        """
        Class constructor
        """

        # Create publisher
        # cspacePub = rospy.Publisher('/cspace_map', OccupancyGrid, queue_size=10)
        
        # Create subscribers
        # gmapSub = rospy.Subscriber('map', OccupancyGrid, gmapCallback)
        # infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
        cspaceSub = rospy.Subscriber('map', OccupancyGrid, cspaceCallback)
        
        # Initialize node
        rospy.init_node("frontier_exploration")
        
        rospy.sleep(1.0)
        rospy.loginfo("frontier_exploration node ready")


    def cspaceCallback(self, msg)
   
    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        rospy.spin()


        
if __name__ == '__main__':
    Navigation().run()
