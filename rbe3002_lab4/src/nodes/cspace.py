#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue
from scripts.map_functions import *


class ConfigSpace:

    
    def __init__(self):
        """
        Class constructor
        """

        self._gmap = OccupancyGrid()
        self.padded_map = OccupancyGrid()
        # self._gmapMetaData = MapMetaData()
        
        # Create publisher
        # cspacePub = rospy.Publisher('/cspace_map', OccupancyGrid, queue_size=10)

        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.Cspace_pub = rospy.Publisher('/cspace_map', GridCells, queue_size=10)
        
        # Create subscribers
        # odomSub = rospy.Subscriber('odom', Odometry, odomCallback)
        # initSub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, initCallback)
        # goalSub = rospy.Subscriber('move_base_simple/goal', PoseStamped, goalCallback)
        # gmapSub = rospy.Subscriber('map', OccupancyGrid, gmapCallback)
        # infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)

        # Create services
        self.cspace_service = rospy.Service('get_padded_map', GetMap, self.calc_cspace)
        self.save_map_service = rospy.Service('save_map', GetMap, self.save_map_callBack)
        
        # Initialize node
        rospy.init_node("cspace")

        rospy.sleep(1.5)
        rospy.loginfo("cspace node ready")


    # @staticmethod
    def request_map(self):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/dynamic_map')
        mapdata = rospy.ServiceProxy('/dynamic_map', GetMap)
        try:
            resp1 = mapdata()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        # self._gmap.data = list(resp1.map.data)
        self._gmap = resp1.map
        
        return resp1.map


    def calc_cspace(self, msg):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space
        """

        ## Constant padding
        padding = 3

        self.request_map()
        ## Threshold map
        # thresh_map = thresholdMap(self._gmap)

        ## Inflate the obstacles where necessary
        # padded_map = thresh_map
        # padded_map.data = list(thresh_map.data)
        padded_map = self._gmap
        padded_map.data = list(self._gmap.data)
        CSpace = []

        for h in range(0, padding):

            if (padding > 0):
                
                for i in range(0, len(padded_map.data)):
                    
                    if (padded_map.data[i] == 100):
                        grid = index_to_grid(padded_map, i)
                        neighbors = neighbors_of_8(padded_map, grid[0], grid[1])
                        
                        for k in neighbors:
                            if k not in CSpace:
                                CSpace.append(k)

                for j in range(0,len(CSpace)):
                    temp_x = CSpace[j][0]
                    temp_y = CSpace[j][1]
                    paddindex = grid_to_index(padded_map, temp_x, temp_y)
                    padded_map.data[paddindex] = 100

        # cspacePub.publish(padded_map)
        self.padded_map = padded_map
        self.padded_map.data = list(padded_map.data)


        # # ## Create a GridCells message and publish it
        for i in range(0,len(CSpace)):
            CSpace[i] = grid_to_world(padded_map, CSpace[i][0], CSpace[i][1])

        gridCell = GridCells()
        gridCell.header.frame_id = 'map'
        gridCell.cell_width = padded_map.info.resolution
        gridCell.cell_height = padded_map.info.resolution
        gridCell.cells = CSpace
        self.Cspace_pub.publish(gridCell)

        return padded_map

    def save_map_callBack(self, msg):
        
        pass

    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # padding = 2
        
        # TEST CASE FOR CSPACE ALONE, without frontier_exploration trigger
        # self.calc_cspace()

        # When do we save map?

        rospy.spin()


        
if __name__ == '__main__':
    ConfigSpace().run()
