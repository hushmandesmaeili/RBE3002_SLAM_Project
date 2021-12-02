#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue
import map_functions as mapf



class FrontierExploration:

    
    def __init__(self):
        """
        Class constructor
        """

        self._frontier_cells = []
        self._frontiers_bin = []
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


    def cspaceCallback(self, msg):
        pass


    def calcFrontier(self, mapdata):

    
    def edgeDetection(self, mapdata):
        """
        Finds all cells that are part of the frontier and stores them
        in _frontier_cells list
        :param mapdata [OccupancyGrid] The map data.
        """
        self._frontier_cells = []

        for i in range(0, len(mapdata.data)):
            grid = mapf.index_to_grid(mapdata, i)
            x = grid[0]
            y = grid[1]

            if (isFrontierCell(mapdata, x, y)):
                _frontier_cells.append(grid)
    

    def segmentFrontiers(self, edgeCells):
        
   
    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        rospy.spin()


        
if __name__ == '__main__':
    Navigation().run()
