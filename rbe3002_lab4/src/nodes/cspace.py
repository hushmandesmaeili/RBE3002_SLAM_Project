#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue
import map_functions as mapf



class ConfigSpace:

    
    def __init__(self):
        """
        Class constructor
        """

        self._gmap = OccupancyGrid()
        self._gmapMetaData = MapMetaData()
        
        # Create publisher
        cspacePub = rospy.Publisher('/cspace_map', OccupancyGrid, queue_size=10)
        
        # Create subscribers
        # odomSub = rospy.Subscriber('odom', Odometry, odomCallback)
        # initSub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, initCallback)
        # goalSub = rospy.Subscriber('move_base_simple/goal', PoseStamped, goalCallback)
        gmapSub = rospy.Subscriber('map', OccupancyGrid, gmapCallback)
        infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
        
        # Initialize node
        rospy.init_node("cspace")

        rospy.sleep(1.0)
        rospy.loginfo("cspace node ready")

    def gmapCallback(self, msg):
        _gmap = msg

    def mapInfoCallback(self, msg):
        _gmapMetaData = msg
    

    # def calc_cspace(self, mapdata, padding):
    #     """
    #     Calculates the C-Space, i.e., makes the obstacles in the map thicker.
    #     Publishes the list of cells that were added to the original map.
    #     :param mapdata [OccupancyGrid] The map data.
    #     :param padding [int]           The number of cells around the obstacles.
    #     :return        [OccupancyGrid] The C-Space
    #     """
        
    #     ## Inflate the obstacles where necessary
        
    #     newmap = mapdata
    #     newmap.data = list(mapdata.data)
    #     CSpace = []

    #     for h in range(0, padding):

    #         if (padding > 0):
                
    #             for i in range(0, len(newmap.data)):
                    
    #                 if (newmap.data[i] == 100):
    #                     grid = mapf.index_to_grid(newmap, i)
    #                     neighbors = neighbors_of_8(newmap, grid[0], grid[1])
                        
    #                     for k in neighbors:
    #                         if k not in CSpace:
    #                             CSpace.append(k)

    #             for j in range(0,len(CSpace)):
    #                 temp_x = CSpace[j][0]
    #                 temp_y = CSpace[j][1]
    #                 paddindex = mapf.grid_to_index(newmap, temp_x, temp_y)
    #                 newmap.data[paddindex] = 100

    #     # # # ## Create a GridCells message and publish it
    #     # for i in range(0,len(CSpace)):
    #     #     CSpace[i] = mapf.grid_to_world(newmap, CSpace[i][0], CSpace[i][1])

    #     # gridCell = GridCells()
    #     # gridCell.header.frame_id = 'map'
    #     # gridCell.cell_width = newmap.info.resolution
    #     # gridCell.cell_height = newmap.info.resolution
    #     # gridCell.cells = CSpace
    #     # self.Cspace_pub.publish(gridCell)

    #     return newmap

    def calc_cspace(self, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space
        """
        
        ## Inflate the obstacles where necessary
        
        padded_map = self._gmap
        padded_map.data = list(self._gmap.data)
        CSpace = []

        for h in range(0, padding):

            if (padding > 0):
                
                for i in range(0, len(padded_map.data)):
                    
                    if (padded_map.data[i] == 100):
                        grid = mapf.index_to_grid(padded_map, i)
                        neighbors = neighbors_of_8(padded_map, grid[0], grid[1])
                        
                        for k in neighbors:
                            if k not in CSpace:
                                CSpace.append(k)

                for j in range(0,len(CSpace)):
                    temp_x = CSpace[j][0]
                    temp_y = CSpace[j][1]
                    paddindex = mapf.grid_to_index(padded_map, temp_x, temp_y)
                    padded_map.data[paddindex] = 100

        cspacePub.publish(padded_map)

        return padded_map

    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        padding = 2
        self.calc_cspace(padding)

        # When do we save map?

        rospy.spin()


        
if __name__ == '__main__':
    ConfigSpace().run()
