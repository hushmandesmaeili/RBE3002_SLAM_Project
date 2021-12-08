#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from rbe3002_lab4.srv import PoseStampedServices
from priority_queue import PriorityQueue
from scripts.map_functions import *
from itertools import groupby, product



class FrontierExploration:

    
    def __init__(self):
        """
        Class constructor
        """

        # self.px = 0
        # self.py = 0

        self._frontier_cells = []
        self._frontiers_bin = []

        # # Stores distance and length for each frontier bin
        # self._frontiers_dict = {}

        # Create publisher
        # cspacePub = rospy.Publisher('/cspace_map', OccupancyGrid, queue_size=10)

        self.Edge_pub = rospy.Publisher('/edge_cells', GridCells, queue_size=10)
        self.centroid_pub = rospy.Publisher('/centroid_cells', GridCells, queue_size=10)
        
        # Create subscribers
        # gmapSub = rospy.Subscriber('map', OccupancyGrid, gmapCallback)
        # infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
        # cspaceSub = rospy.Subscriber('map', OccupancyGrid, self.getCSpace)

        # odomSub = rospy.Subscriber('/odom', Odometry, self.update_odometry)

        # Create services
        self.getFrontier_service = rospy.Service('get_frontier', PoseStampedServices, self.getFrontier)
        
        # Initialize node
        rospy.init_node("frontier_exploration")
        
        rospy.sleep(2.0)
        rospy.loginfo("frontier_exploration node ready")


    def getFrontier(self, msg):

        x_start = msg.pose.pose.position.x
        y_start = msg.pose.pose.position.y

        frontier_to_explore = False

        print("Get CSpace")
        map = self.getCSpace()

        frontiersPriorityQueue = PriorityQueue()
        
        print("Edge detection")

        # mapTest = OccupancyGrid()

        # mapTest.info.height = 4
        # mapTest.info.width = 4
        # mapTest.data = [100, 100, 100, 100, 100, -1,-1,100,100,-1,-1,100,100,0,0,100]

        self.edgeDetection(map)
        # self.edgeDetection(mapTest)
        # print(self._frontier_cells)
        print("Segment")
        self.segmentFrontiers(map)
        # print(self._frontiers_bin)

        centroid_gridlist = []

        for i in range(len(self._frontiers_bin)):

            centroid = self.calcCentroid(self._frontiers_bin[i])
            centroid_gridlist.append(centroid)

            # x_start = self.px
            # y_start = self.py
            x_goal = centroid[0]
            y_goal = centroid[1]
            distance = euclidean_distance(x_start, y_start, x_goal, y_goal)
            
            length = self.calcLength(self._frontiers_bin[i])
            # print(length)

            # priority = 10*distance/length
            priority = 0.75*0.1*distance + 0.25*length
            # print(grid_to_world(map, *centroid), centroid, distance, length, priority)

            if (map.data[grid_to_index(map, *centroid)] != -1 and map.data[grid_to_index(map, *centroid)] != 100):
                frontiersPriorityQueue.put(centroid, priority)

            if (not frontier_to_explore and length > 1):
                frontier_to_explore = True

        ## Print centroid gridcells
        pointList = gridList_to_pointList(map, centroid_gridlist)
        
        gridCell = GridCells()
        gridCell.header.frame_id = 'map'
        gridCell.cell_width = map.info.resolution
        gridCell.cell_height = map.info.resolution
        gridCell.cells = pointList

        self.centroid_pub.publish(gridCell)

        priorityFrontier = frontiersPriorityQueue.get()
        # print(priorityFrontier)
        
        priorityFrontier_PoseStamped = PoseStamped()
        priorityFrontier_PoseStamped.pose.position = grid_to_world(map, *priorityFrontier)
        # print(priorityFrontier_PoseStamped)

        resp = {'pose': priorityFrontier_PoseStamped, 'frontiers': frontier_to_explore}

        # return priorityFrontier_PoseStamped
        return resp


    def getCSpace(self):
        rospy.loginfo("Requesting the cspace map")
        rospy.wait_for_service('get_padded_map')
        mapdata = rospy.ServiceProxy('get_padded_map', GetMap)
        try:
            resp1 = mapdata()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        return resp1.map

        # pass

        
    def calcCentroid(self, cells):
        
        # frontier_cells = []

        x_coordinate = [c[0] for c in cells]
        y_coordinate = [c[1] for c in cells]

        n = len(cells)
        
        centroid_x = sum(x_coordinate)/n
        centroid_y = sum(y_coordinate)/n

        # frontier_cells.append((centroid_x, centroid_y))

        # return frontier_cells
        return (centroid_x, centroid_y)

    
    def calcLength(self, bin):

        return len(bin)

    
    def edgeDetection(self, mapdata):
        """
        Finds all cells that are part of the frontier and stores them
        in _frontier_cells list
        :param mapdata [OccupancyGrid] The map data.
        """
        self._frontier_cells = []

        for i in range(0, len(mapdata.data)):
            grid = index_to_grid(mapdata, i)
            x = grid[0]
            y = grid[1]

            if (isFrontierCell(mapdata, x, y)):
                self._frontier_cells.append(grid)

        pointList = gridList_to_pointList(mapdata, self._frontier_cells)
        
        gridCell = GridCells()
        gridCell.header.frame_id = 'map'
        gridCell.cell_width = mapdata.info.resolution
        gridCell.cell_height = mapdata.info.resolution
        gridCell.cells = pointList

        self.Edge_pub.publish(gridCell)
    
    
    def segmentFrontiers(self, mapdata):

        frontier_cells = self._frontier_cells
        
        tuple_pairs = [sorted(pair) for pair in product(frontier_cells, repeat=2) if euclidean_distance_tups(*pair) <= 1.5]

        result_dict = {t: {t} for t in frontier_cells}
        for t1, t2 in tuple_pairs:
            # "Unify" these tuple's groups
            result_dict[t1] |= result_dict[t2]
            result_dict[t2] = result_dict[t1]

        result = [list((next(g))) for k, g in groupby(sorted(result_dict.values(), key=id), id)]
        self._frontiers_bin = result
                    

    # def update_odometry(self, msg):
    #     """
    #     Updates the current pose of the robot.
    #     This method is a callback bound to a Subscriber.
    #     :param msg [Odometry] The current odometry information.
    #     """
    #     ### REQUIRED CREDIT
    #     self.px = msg.pose.pose.position.x
    #     self.py = msg.pose.pose.position.y
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # self.getFrontier()
        rospy.spin()

        
if __name__ == '__main__':
    FrontierExploration().run()
