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
# from collections import defaultdict
from itertools import groupby, product



class FrontierExploration:

    
    def __init__(self):
        """
        Class constructor
        """

        self.px = 0
        self.py = 0

        self._frontier_cells = []
        self._frontiers_bin = []

        # # Stores distance and length for each frontier bin
        # self._frontiers_dict = {}

        # Create publisher
        # cspacePub = rospy.Publisher('/cspace_map', OccupancyGrid, queue_size=10)

        self.Edge_pub = rospy.Publisher('/edge_cells', GridCells, queue_size=10)
        
        # Create subscribers
        # gmapSub = rospy.Subscriber('map', OccupancyGrid, gmapCallback)
        # infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
        # cspaceSub = rospy.Subscriber('map', OccupancyGrid, self.getCSpace)

        odomSub = rospy.Subscriber('/odom', Odometry, self.update_odometry)

        # Create services
        self.getFrontier_service = rospy.Service('get_frontier', PoseStampedServices, self.getFrontier)
        
        # Initialize node
        rospy.init_node("frontier_exploration")
        
        rospy.sleep(1.0)
        rospy.loginfo("frontier_exploration node ready")


    def getFrontier(self):

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
        print(self._frontiers_bin)

        # for i in range(len(self._frontiers_bin)):

        #     centroid = self.calcCentroid(self._frontiers_bin[i])

        #     x_start = self.px
        #     y_start = self.py
        #     x_goal = centroid[0]
        #     y_goal = centroid[1]
        #     distance = euclidean_distance(x_start, y_start, x_goal, y_goal)
            
        #     length = self.calcLength()

        #     priority = distance/length

        #     frontiersPriorityQueue.put(centroid, priority)
        
        # return frontiersPriorityQueue.get()


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
        
        ## Code taken and adapted from GeeksForGeeks article 
        # Python — Group Adjacent Coordinates
        # https://www.geeksforgeeks.org/python-group-adjacent-coordinates/amp/
        
        # Code adapted to do diagonal neighbors as well as adjacent
        # Code fixed as well to format the list of lists and store in global bin

        frontier_cells = self._frontier_cells
        
        tuple_pairs = [sorted(pair) for pair in product(frontier_cells, repeat=2) if euclidean_distance(*pair) <= 1.5]

        result_dict = {t: {t} for t in frontier_cells}
        for t1, t2 in tuple_pairs:
            # "Unify" these tuple's groups
            result_dict[t1] |= result_dict[t2]
            result_dict[t2] = result_dict[t1]

        result = [list((next(g))) for k, g in groupby(sorted(result_dict.values(), key=id), id)]
        self._frontiers_bin = result
        # frontier_cells.sort(key=lambda y: y[1])

        # tempBin = defaultdict(lambda : [])
        # tempBin[0] = []
        # counter = 0

        # for frontier_cell in frontier_cells:
        #     print(frontier_cell)

        #     if frontier_cell == frontier_cells[0]:
                
        #         tempBin[0].append(frontier_cell)

        #     else:

        #         neighbors = neighbors_of_8_unknown(mapdata, *frontier_cell)

        #         for key in tempBin.keys():

        #             if not set(neighbors).isdisjoint(set(tempBin.get(key))):

        #                 tempBin[key].append(frontier_cell)

        #             else:

        #                 counter += 1
        #                 tempBin[counter].append(frontier_cell)

        # for key in tempBin.keys():

        #     self._frontiers_bin.append(tempBin[key])
        


        # man_tups = [sorted(sub) for sub in product(frontier_cells, repeat = 2)
        #                                     if euclidean_distance(*sub) <= 1.5]
        

        # res_dict = {ele: {ele} for ele in frontier_cells}

        # for tup1, tup2 in man_tups:
        #     res_dict[tup1] |= res_dict[tup2]
        #     res_dict[tup2] = res_dict[tup1]
        

        # res = [[*next(val)] for key, val in groupby(sorted(res_dict.values(), key = id), id)]

        # self._frontiers_bin = res
        
        # self._frontiers_bin = []

        # neighborList = []
        # tempBin = []

        # while self._frontier_cells:

        #     for point in self._frontier_cells:
                
        #         self._frontier_cells.remove(point)
        #         tempBin.append(point)
                
        #         neighborList = neighbors_of_8_unknown(mapdata, point[0], point[1])
                

        #         for neighbor in neighborList:
        #             tempBin.append(neighbor)
                
        #         for neighbor in neighborList:
        #             self.segmentFrontiers(mapdata, neighborList)

        #     if not neighborList:
        #         differenceBin = list(set(tempBin) - set(self._frontier_cells))
        #         self._frontiers_bin.append(differenceBin)
        #         tempBin = []
        #         differenceBin = []
                    

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        self.getFrontier()
        rospy.spin()

        
if __name__ == '__main__':
    FrontierExploration().run()
