#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from rbe3002_lab4.srv import FrontierReachable, GetFullPlan
from priority_queue import PriorityQueue
from scripts.map_functions import *



class PathPlanner:

    
    def __init__(self):
        """
        Class constructor
        """

        self.extra_cost = 2

        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new sresoltion - x_anchord calls self.plan_path() when a message is received
        self.plan_service = rospy.Service('plan_path', GetPlan, self.plan_path)
        self.full_plan_service = rospy.Service('plan_path_full', GetFullPlan, self.plan_path_full)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.Cspace_pub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.expanded_cells_pub = rospy.Publisher('/path_planner/expanded_cells', GridCells, queue_size=10)
        self.frontier_pub = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)
        self.visited_pub = rospy.Publisher('/path_planner/visited', GridCells, queue_size=10)
        self.path_pub = rospy.Publisher('/path_planner/path', GridCells, queue_size=10)
        # Create Services
        self.isFrontierReachable_service = rospy.Service('frontier_reachable', FrontierReachable, self.isFrontierReachable)
        ## Initialize the request counter
        # TODO
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")


    def getCSpace(self):
        rospy.loginfo("Requesting the cspace map")
        rospy.wait_for_service('get_padded_map')
        mapdata = rospy.ServiceProxy('get_padded_map', GetMap)
        try:
            resp1 = mapdata()
            return resp1.map
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


    def a_star(self, mapdata, start, goal):
        """
        :param mapdata [OccupancyGrid] The map data.
        :param start [PoseStamped] The start point of the path
        :param goal [PoseStamped] The goal point of the path

        """
        ### REQUIRED CREDIT
        #rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start.pose.position[0], start.pose.position[1], goal.pose.position[0], goal.pose.position[1]))
        start = PoseStamped_to_GridCoor(mapdata, start)
        goal = PoseStamped_to_GridCoor(mapdata, goal)
        
        frontier = PriorityQueue()
        frontier.put(start, 0)         ## (OBJECT, PRIORITY)

        came_from = {}                 ##(KEY, VALUE)
        cost_so_far = {}                       ## GRAPH IS MAPDATA
        came_from[start] = None                
        cost_so_far[start] = 0
        visited = []

        while not frontier.empty():
            # rospy.sleep(0.01)

            current = frontier.get()    ## FRONTIER CURRENT, START, AND GOAL ARE Grid Coordinates 
            
            if current == goal:
                visited.append(current)
                # self.publishFrontier(mapdata, frontier)
                # self.publishVisited(mapdata, visited)
                break

            for next in neighbors_of_8(mapdata, current[0], current[1]):
                current_pose_stamped = Grid_to_PoseStamped(mapdata, current)
                next_pose_stamped = Grid_to_PoseStamped(mapdata, next)

                new_cost = cost_so_far[current] + self.find_cost(current_pose_stamped, next_pose_stamped)

                if next not in cost_so_far or new_cost < cost_so_far[next]:

                    cspace_neighbors_len = len(neighbors_of_8_cspace(mapdata, next[0], next[1]))

                    cost_so_far[next] = new_cost

                    goal_pose_stamped = Grid_to_PoseStamped(mapdata, goal)
                    priority = new_cost + 0.5*self.heuristic(goal_pose_stamped, next_pose_stamped) + 4*cspace_neighbors_len

                    frontier.put(next, priority)
                    came_from[next] = current

            # visited.append(current)
            # self.publishFrontier(mapdata, frontier)
            # self.publishVisited(mapdata, visited)

        path = PathPlanner.CameFrom_to_Path(mapdata, start, goal, came_from)
        path.reverse()

        self.publishPath(mapdata, path)
        # optimized = self.optimize_path(path)
        # self.publishPath(mapdata, optimized)
        
        # path_pose_stamped = self.path_to_message(mapdata, path)

        # return path_pose_stamped
        return path


    def publishFrontier(self, mapdata, priority_queue):
        frontier_gridcell = GridCells()
        frontier_gridcell.header.frame_id = 'map'
        frontier_gridcell.cell_width = mapdata.info.resolution
        frontier_gridcell.cell_height = mapdata.info.resolution

        gridList = []

        for elem in priority_queue.get_queue():
            gridList.append(elem[1])

        pointList = PathPlanner.gridList_to_pointList(mapdata, gridList)

        frontier_gridcell.cells = pointList

        self.frontier_pub.publish(frontier_gridcell)


    def publishVisited(self, mapdata, visited_list):
        visited_gridcell = GridCells()
        visited_gridcell.header.frame_id = 'map'
        visited_gridcell.cell_width = mapdata.info.resolution
        visited_gridcell.cell_height = mapdata.info.resolution

        gridList = visited_list
        pointList = gridList_to_pointList(mapdata, gridList)

        visited_gridcell.cells = pointList

        self.visited_pub.publish(visited_gridcell)


    def publishPath(self, mapdata, path_list):
        path_gridcell = GridCells()
        path_gridcell.header.frame_id = 'map'
        path_gridcell.cell_width = mapdata.info.resolution
        path_gridcell.cell_height = mapdata.info.resolution

        gridList = path_list
        pointList = gridList_to_pointList(mapdata, gridList)

        path_gridcell.cells = pointList

        self.path_pub.publish(path_gridcell)


    @staticmethod
    def CameFrom_to_Path(mapdata, start, goal, came_from):
        path = []
        path.append(goal)

        current = goal

        while start not in path:
            path.append(came_from[current])
            current = came_from[current]

        return path
        

    def find_cost(self, first, second):
        ### CALCULATES THE COST TO GO FROM THE CURRENT NODE TO THE NEXT
        x_current = first.pose.position.x
        y_current = first.pose.position.y
        x_goal = second.pose.position.x
        y_goal = second.pose.position.y
        cost = euclidean_distance(x_current, y_current, x_goal, y_goal)
        return cost


    def heuristic(self, goal, point):
        ### CALCULATES PREDICTED COST OF GETTING TO FROM GOAL TO A POINT
        x_point = point.pose.position.x
        y_point = point.pose.position.y
        x_goal = goal.pose.position.x
        y_goal = goal.pose.position.y
        predicted_cost = abs(x_goal - x_point) + abs(y_goal - y_point)
        return predicted_cost


    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        path_optimized = []

        # Append start of path
        path_optimized.append(path[0])

        for i in range(1, len(path)):
            if (i == len(path) - 1):
                path_optimized.append(path[i])
            else:

                coor_prev = path[i - 1]
                coor_current = path[i]
                coor_next = path[i + 1]
                
                x_prev = coor_prev[0]
                y_prev = coor_prev[1]
                x_current = coor_current[0]
                y_current = coor_current[1]
                x_next = coor_next[0]
                y_next = coor_next[1]

                if ((not(x_current == x_prev or y_current == y_prev) or not(x_current == x_next or y_current == y_next)) and ((not(abs(x_current - x_prev) == 1 and abs(y_current - y_prev) == 1)) or (not(abs(x_current - x_next) == 1 and abs(y_current - y_next) == 1)))):
                    path_optimized.append(coor_current)

        return path_optimized

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        # path_message = Path()
        # path_message.poses = path
        # return path_message
        rospy.loginfo("Returning a Path message")

        poseList = []
        path_message = Path()
        path_message.header.frame_id = 'map'

        for grid in path:
            point = Grid_to_PoseStamped(mapdata, grid)
            poseList.append(point)

        path_message.poses = poseList

        return path_message


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        # mapdata = PathPlanner.request_map()
        print('Plan Path')
        print('Getting CSpace')
        mapdata = self.getCSpace()
        if mapdata is None:
            return Path()

        start = msg.start
        goal = msg.goal
        print('Getting path')
        path = self.a_star(mapdata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)


    def plan_path_full(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        # mapdata = PathPlanner.request_map()
        print('Plan Full Path')
        print('Getting CSpace')
        mapdata = self.getCSpace()
        if mapdata is None:
            return Path()
            
        start = msg.start
        goal = msg.goal
        print('Getting path')
        path = self.a_star(mapdata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message

        path_full_message = self.path_to_message(mapdata, path)
        path_optimized_message = self.path_to_message(mapdata, waypoints)

        resp = {'full': path_full_message, 'optimized': path_optimized_message}
        print(resp)

        return resp


    def isFrontierReachable(self, msg):

        poseFrontier = msg.poseFrontier

        print('Plan isFrontier')
        print('Getting CSpace')
        mapdata = self.getCSpace()

        if mapdata is None:
            return Path()

        start = msg.poseStart

        print('Getting path')
        
        try:
            path = self.a_star(mapdata, start, poseFrontier)
            print('Reachable')
            return True
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            print('Unreachable')
            return False


    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
