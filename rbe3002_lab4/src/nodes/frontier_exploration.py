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

        self.px = 0
        self.py = 0

        self._frontier_cells = []
        self._frontiers_bin = []

        # # Stores distance and length for each frontier bin
        # self._frontiers_dict = {}

        # Create publisher
        # cspacePub = rospy.Publisher('/cspace_map', OccupancyGrid, queue_size=10)
        
        # Create subscribers
        # gmapSub = rospy.Subscriber('map', OccupancyGrid, gmapCallback)
        # infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
        cspaceSub = rospy.Subscriber('map', OccupancyGrid, cspaceCallback)

        odomSub = rospy.Subscriber('/odom', Odometry, self.update_odometry)

        # Create services
        self.getFrontier_service = rospy.Service('get_frontier', PoseStamped, self.getFrontier)
        
        # Initialize node
        rospy.init_node("frontier_exploration")
        
        rospy.sleep(1.0)
        rospy.loginfo("frontier_exploration node ready")


    def getFrontier(self, msg):

        map = self.getCSpace()

        frontiersPriorityQueue = PriorityQueue()
        
        self.edgeDetection(map)
        self.segmentFrontiers(map)

        for i in range(len(self._frontiers_bin)):

            centroid = self.calcCentroid(self._frontiers_bin[i])

            x_start = self.px
            y_start = self.py
            x_goal = centroid[0]
            y_goal = centroid[1]
            distance = mapf.euclidean_distance(x_start, y_start, x_goal, y_goal)
            
            length = self.calcLength()

            priority = distance/length

            frontiersPriorityQueue.put(centroid, priority)
        
        return frontiersPriorityQueue.get()


    def getCSpace(self):
        rospy.loginfo("Requesting the cspace map")
        rospy.wait_for_service('/get_padded_map')
        mapdata = rospy.ServiceProxy('/get_padded_map', GetMap)
        try:
            resp1 = mapdata()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        return resp1.map

        pass

    def getDistance(self, mapdata, goal):
        """
        Returns distance to a given point
        """
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
                    

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw
    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        rospy.spin()


        
if __name__ == '__main__':
    Navigation().run()
