#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue



class PathPlanner:

    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new sresoltion - x_anchord calls self.plan_path() when a message is received
        self.plan_service = rospy.Service('plan_path', GetPlan, self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.Cspace_pub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.expanded_cells_pub = rospy.Publisher('/path_planner/expanded_cells', GridCells, queue_size=10)
        self.frontier_pub = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)
        self.visited_pub = rospy.Publisher('/path_planner/visited', GridCells, queue_size=10)
        self.path_pub = rospy.Publisher('/path_planner/path', GridCells, queue_size=10)
        ## Initialize the request counter
        # TODO
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")



    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        index = y * mapdata.info.width + x
        
        return index
        

    @staticmethod
    def index_to_grid(mapdata, i):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        y = math.floor(i / mapdata.info.width)
        x = i - y * mapdata.info.width
        x = int(x)
        y = int(y)

        return (x, y)


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        distance = (math.sqrt((x2 - x1)**2 + (y2 - y1)**2))
        return distance
        
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        # wp.x and wp.y are the world coordinates
        # gc.x and gc.y are the grid coordinates
        # resolution is the map resolution
        # origin.x and origin.y are the position of the origin in the world
        wp_x = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        wp_y = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
        worldPoint = Point()
        worldPoint.x = wp_x
        worldPoint.y = wp_y

        return worldPoint
        


        
    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        # wc.x and wc.y are the world coordinates
        # gc.x and gc.y are the grid coordinates
        # resolution is the map resolution
        # origin.x and origin.y are the position of the origin in the world
        # ADAPT THIS CODE TO THE TEMPLATE - DO NOT COPY-PASTE
        
        resolution = mapdata.info.resolution

        origin_x = mapdata.info.origin.position.x
        origin_y = mapdata.info.origin.position.y

        gc_x = int((wp.x - origin_x) / resolution)
        gc_y = int((wp.y - origin_y) / resolution)

        return (gc_x, gc_y)



        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        pass

    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """

        index = PathPlanner.grid_to_index(mapdata,x,y)
        
        if (x >= 0 and x < mapdata.info.width and y >= 0 and y < mapdata.info.height):
            if(mapdata.data[index] == 0):
                return True
            else: 
                return False
        else:
            return False
                     

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        walkable_neighbours = []
        
        if (PathPlanner.is_cell_walkable(mapdata,x,y-1)):
            walkable_neighbours.append((x,y-1))
        if (PathPlanner.is_cell_walkable(mapdata,x,y+1)):
            walkable_neighbours.append((x,y+1))

        if (PathPlanner.is_cell_walkable(mapdata,x-1,y)):
            walkable_neighbours.append((x-1,y))
        if (PathPlanner.is_cell_walkable(mapdata,x+1,y)):
            walkable_neighbours.append((x+1,y))

        return walkable_neighbours


    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        walkable_neighbours = []
        
        for i in range(x - 1, x + 2):
            for j in range(y - 1, y + 2):
                if (not(i == x and j == y)):
                    if (PathPlanner.is_cell_walkable(mapdata, i, j)):
                        walkable_neighbours.append((i, j))

        return walkable_neighbours
        
    
    
    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/static_map')
        mapdata = rospy.ServiceProxy('/static_map', GetMap)
        try:
            resp1 = mapdata()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return resp1.map



    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space
        """
        
        ## Inflate the obstacles where necessary
        
        newmap = mapdata
        newmap.data = list(mapdata.data)
        CSpace = []

        for h in range(0, padding):

            if (padding > 0):
                
                for i in range(0, len(newmap.data)):
                    
                    if (newmap.data[i] == 100):
                        grid = PathPlanner.index_to_grid(newmap, i)
                        neighbors = PathPlanner.neighbors_of_8(newmap, grid[0], grid[1])
                        
                        for k in neighbors:
                            if k not in CSpace:
                                CSpace.append(k)

                for j in range(0,len(CSpace)):
                    temp_x = CSpace[j][0]
                    temp_y = CSpace[j][1]
                    paddindex = PathPlanner.grid_to_index(newmap, temp_x, temp_y)
                    newmap.data[paddindex] = 100

        # # ## Create a GridCells message and publish it
        for i in range(0,len(CSpace)):
            CSpace[i] = PathPlanner.grid_to_world(newmap, CSpace[i][0], CSpace[i][1])

        gridCell = GridCells()
        gridCell.header.frame_id = 'map'
        gridCell.cell_width = newmap.info.resolution
        gridCell.cell_height = newmap.info.resolution
        gridCell.cells = CSpace
        self.Cspace_pub.publish(gridCell)

        return newmap


    def a_star(self, mapdata, start, goal):
        """
        :param mapdata [OccupancyGrid] The map data.
        :param start [PoseStamped] The start point of the path
        :param goal [PoseStamped] The goal point of the path
        """
        ### REQUIRED CREDIT
        #rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start.pose.position[0], start.pose.position[1], goal.pose.position[0], goal.pose.position[1]))
        start = PathPlanner.PoseStamped_to_GridCoor(mapdata, start)
        goal = PathPlanner.PoseStamped_to_GridCoor(mapdata, goal)
        
        frontier = PriorityQueue()
        frontier.put(start, 0)         ## (OBJECT, PRIORITY)

        came_from = {}                 ##(KEY, VALUE)
        cost_so_far = {}                       ## GRAPH IS MAPDATA
        came_from[start] = None                
        cost_so_far[start] = 0
        visited = []

        while not frontier.empty():
            rospy.sleep(0.01)

            current = frontier.get()    ## FRONTIER CURRENT, START, AND GOAL ARE Grid Coordinates 
            
            if current == goal:
                visited.append(current)
                self.publishFrontier(mapdata, frontier)
                self.publishVisited(mapdata, visited)
                break

            for next in self.neighbors_of_8(mapdata, current[0], current[1]):
                current_pose_stamped = PathPlanner.Grid_to_PoseStamped(mapdata, current)
                next_pose_stamped = PathPlanner.Grid_to_PoseStamped(mapdata, next)

                new_cost = cost_so_far[current] + self.find_cost(current_pose_stamped, next_pose_stamped)

                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost

                    goal_pose_stamped = PathPlanner.Grid_to_PoseStamped(mapdata, goal)
                    priority = new_cost + self.heuristic(goal_pose_stamped, next_pose_stamped)

                    frontier.put(next, priority)
                    came_from[next] = current

            visited.append(current)
            self.publishFrontier(mapdata, frontier)
            self.publishVisited(mapdata, visited)

        path = PathPlanner.CameFrom_to_Path(mapdata, start, goal, came_from)
        path.reverse()

        # self.publishPath(mapdata, path)
        # optimized = self.optimize_path(path)
        # self.publishPath(mapdata, optimized)
        
        path_pose_stamped = self.path_to_message(mapdata, path)

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
        pointList = PathPlanner.gridList_to_pointList(mapdata, gridList)

        visited_gridcell.cells = pointList

        self.visited_pub.publish(visited_gridcell)


    def publishPath(self, mapdata, path_list):
        path_gridcell = GridCells()
        path_gridcell.header.frame_id = 'map'
        path_gridcell.cell_width = mapdata.info.resolution
        path_gridcell.cell_height = mapdata.info.resolution

        gridList = path_list
        pointList = PathPlanner.gridList_to_pointList(mapdata, gridList)

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
        

    @staticmethod
    def gridList_to_pointList(mapdata, gridList):
        pointList = []

        for elem in gridList:
            pointList.append(PathPlanner.grid_to_world(mapdata, elem[0], elem[1]))

        return pointList


    @staticmethod
    def PoseStamped_to_GridCells(mapdata, pStamped):
        gridcell = GridCells()
        gridcell.header.frame_id = 'map'
        gridcell.cell_width = mapdata.info.resolution
        gridcell.cell_height = mapdata.info.resolutionf

        worldPoint = PathPlanner.PoseStamped_to_WorldPoint(pStamped)
        gridcell.cells = worldPoint

        return gridcell

    @staticmethod
    def Grid_to_PoseStamped(mapdata, grid):
        pose = PoseStamped()
        grid_x = grid[0]
        grid_y = grid[1]

        worldPoint = PathPlanner.grid_to_world(mapdata, grid_x, grid_y)
        
        pose.pose.position.x = worldPoint.x
        pose.pose.position.y = worldPoint.y

        return pose


    @staticmethod
    def PoseStamped_to_GridCoor(mapdata, pStamped):
        worldPoint = PathPlanner.PoseStamped_to_WorldPoint(pStamped)

        grid_cell = PathPlanner.world_to_grid(mapdata, worldPoint)

        return grid_cell


    @staticmethod
    def PoseStamped_to_WorldPoint(pStamped):
        worldPoint = Point()
        worldPoint.x = pStamped.pose.position.x
        worldPoint.y = pStamped.pose.position.y

        return worldPoint
    

    def find_cost(self, first, second):
        ### CALCULATES THE COST TO GO FROM THE CURRENT NODE TO THE NEXT
        x_current = first.pose.position.x
        y_current = first.pose.position.y
        x_goal = second.pose.position.x
        y_goal = second.pose.position.y
        cost = self.euclidean_distance(x_current, y_current, x_goal, y_goal)
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
            point = PathPlanner.Grid_to_PoseStamped(mapdata, grid)
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
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        # start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        # goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        start = msg.start
        goal = msg.goal
        path = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)

    
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        # mapdata = self.request_map()

        # final_map = self.calc_cspace(mapdata, 1)
        # start = PoseStamped()
        # start.pose.position.x = -4.16
        # start.pose.position.y = 5.24

        # end = PoseStamped()
        # end.pose.position.x = 5.38
        # end.pose.position.y = -4.25
        # self.a_star(final_map, start, end)
        # PathPlanner.index_to_grid(mapdata, 74)
        
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()