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
        
    def calcCentroid(self,mapdata, cells):
        
       frontier_cells = []

       x_coordinate = [c[0] for c in cells]
       y_coordinate = [c[1] for c in cells]

       n = len(cells)
       
       centroid_x = sum(x_coordinate)/n
       centroid_y = sum(y_coordinate)/n

       append.frontier_cells([centroid_x,centroid_y])

       return frontier_cells

    
    def calcLength(self, mapdata, bin):

        return len(bin)

    
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

            if (mapf.isFrontierCell(mapdata, x, y)):
                _frontier_cells.append(grid)
    

    def segmentFrontiers(self, mapdata, list):
        
        self._frontiers_bin = []
        neighborList = []
        tempBin = []

        while self._frontier_cells:

            for point in self._frontier_cells:
                
                self._frontier_cells.remove(point)
                tempBin.append(point)
                
                neighborList = mapf.neighbors_of_8_unknown(mapdata, point[0], point[1])
                

                for neighbor in neighborList:
                    tempBin.append(neighbor)
                
                for neighbor in neighborList:
                    self.segmentFrontiers(mapdata, neighborList)

            if not neighborList:
                differenceBin = list(set(tempBin) - set(self._frontier_cells))
                self._frontiers_bin.append(differenceBin)
                tempBin = []
                differenceBin = []
                    
                    


   
    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        rospy.spin()


        
if __name__ == '__main__':
    Navigation().run()
