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
        # print(index)
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
        # print(x, y)
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
        CSpace = []

        if (padding > 0):
            # for i in range(0, len(mapdata.data)):
            for i in range(0, len(mapdata.data)):
                if (mapdata.data[i] == 100):
                    grid = PathPlanner.index_to_grid(mapdata, i)
                    CSpace.append((PathPlanner.neighbors_of_8(mapdata, grid[0], grid[1])))

                # for j in range(0, len(CSpace)):
            print(len(CSpace))
            # print(CSpace)
            for j in range(0,len(CSpace)):
                print(CSpace[j])
                # temp_x = (CSpace[j])[0]
                # temp_y = (CSpace[j])[1]
        #         temp_x = 0
        #         temp_y = 0
        #         # print(temp_x, temp_y)
        #         paddindex = PathPlanner.grid_to_index[newmap, temp_x, temp_y]
        #         newmap.data[paddindex] = 100

        #     padding -= 1
        #     if (not(padding == 0)):
        #         calc_cspace(newmap, n - 1)

        # ## Create a GridCells message and publish it
        # self.expanded_cells_pub.publish(newmap)

    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))


    
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
        # worldPoint = Point()
        # worldPoint.x = -3.95
        # worldPoint.y = -4.66

        # grid = PathPlanner.world_to_grid(mapdata, worldPoint)
        # #print(grid)

        # #print(PathPlanner.is_cell_walkable(mapdata,grid[0],grid[1]))

        # PathPlanner.neighbors_of_8(mapdata,grid[0],grid[1])

        self.calc_cspace(mapdata, 2)
        # PathPlanner.index_to_grid(mapdata, 74)
        
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
