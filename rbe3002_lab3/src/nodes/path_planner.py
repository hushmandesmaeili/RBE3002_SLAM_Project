#!/usr/bin/env python

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



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
        print(index)
        return index
        



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

        
        print(walkable_neighbours)
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
        :param padding [iresoltion - x_anchorh cell in the occupancy grid """
        ## Inflate the obstacles where necessary
        # TODO
        ## Create a GridCells message and publish it
        # TODO
        ## Return the C-space
        pass


    
    def a_star(self, mapdata, start, goal):
        """
        :param mapdata [OccupancyGrid] The map data.
        :param start [PoseStamped] The start point of the path
        :param goal [PoseStamped] The goal point of the path

        """
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        frontier = PriorityQueue()
        frontier.put(start, 0)         ## (OBJECT, PRIORITY)
        self.frontier_pub.publish(frontier)
        came_from = {}                 ##(KEY, VALUE)
        cost_so_far = {}                       ## GRAPH IS MAPDATA
        came_from[start] = None                
        cost_so_far[start] = 0
        while not frontier.empty():
            current = frontier.get()    ## CURRENT, START, AND GOAL ARE POSESTAMPED MESSAGES
            self.frontier_pub(frontier)
            if current == goal:
                break
            for next in self.neighbors_of_8(mapdata, current.pose.position.x, current.pose.position.y):
                new_cost = cost_so_far[current] + self.find_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
                    self.visited_pub.publish(came_from)
        return frontier

    
    def find_cost(first, second):
        ### CALCULATES THE COST TO GO FROM THE CURRENT NODE TO THE NEXT
        x_current = first.pose.position.x
        y_current = first.pose.position.y
        x_goal = second.pose.position.x
        y_goal = second.pose.position.y
        cost = euclidean_distance(x_current, y_current, x_goal, y_goal)
        return cost


    def heuristic(goal, point):
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

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")


        
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
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)

        


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        mapdata = self.request_map()
        worldPoint = Point()
        worldPoint.x = -3.95
        worldPoint.y = -4.66

        grid = PathPlanner.world_to_grid(mapdata, worldPoint)
        #print(grid)

        #print(PathPlanner.is_cell_walkable(mapdata,grid[0],grid[1]))

        PathPlanner.neighbors_of_8(mapdata,grid[0],grid[1])
        
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
