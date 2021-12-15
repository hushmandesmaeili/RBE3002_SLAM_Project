#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue



def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        index = y * mapdata.info.width + x
        
        return index

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

def euclidean_distance_tups(tup1, tup2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        x1 = tup1[0]
        y1 = tup1[1]
        x2 = tup2[0]
        y2 = tup2[1]
        distance = euclidean_distance(x1, y1, x2, y2)
        return distance

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

def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        pass

    

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

    index = grid_to_index(mapdata,x,y)
    
    if (x >= 0 and x < mapdata.info.width and y >= 0 and y < mapdata.info.height):
        if (mapdata.data[index] == 0):
            return True
        else: 
            return False
    else:
        return False
                     

def neighbors_of_4(mapdata, x, y):
    """
    Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [[(int,int)]]   A list of walkable 4-neighbors.
    """
    walkable_neighbours = []
    
    if (is_cell_walkable(mapdata, x, y-1)):
        walkable_neighbours.append((x, y-1))
    if (is_cell_walkable(mapdata, x, y+1)):
        walkable_neighbours.append((x, y+1))

    if (is_cell_walkable(mapdata, x-1, y)):
        walkable_neighbours.append((x-1, y))
    if (is_cell_walkable(mapdata,x+1, y)):
        walkable_neighbours.append((x+1, y))

    return walkable_neighbours



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
                if (is_cell_walkable(mapdata, i, j)):
                    walkable_neighbours.append((i, j))

    return walkable_neighbours

def neighbors_of_8_unknown(mapdata, x, y):
    """
    Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [[(int,int)]]   A list of walkable 8-neighbors.
    """
    unknown_neighbours = []
    
    for i in range(x - 1, x + 2):
        for j in range(y - 1, y + 2):
            if (not(i == x and j == y)):
                index = grid_to_index(mapdata,i,j)
                if (mapdata.data[index] == -1):
                    unknown_neighbours.append((i, j))

    return unknown_neighbours

def neighbors_of_8_cspace(mapdata, x, y):
    """
    Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [[(int,int)]]   A list of walkable 8-neighbors.
    """
    cspace_neighbours = []
    
    for i in range(x - 1, x + 2):
        for j in range(y - 1, y + 2):
            if (not(i == x and j == y)):
                index = grid_to_index(mapdata, i,j)
                if (mapdata.data[index] == 100):
                    cspace_neighbours.append((i, j))

    return cspace_neighbours

def isFrontierCell(mapdata, x, y):

    # cell = index_to_grid(mapdata, index)

    # x = cell[0]
    # y = cell[1]

    ## FIX RETURN LOGIX !!!!

    isTrue = False
    indexCurrent = grid_to_index(mapdata, x, y)

    if (mapdata.data[indexCurrent] == 0):
        for i in range(x - 1, x + 2):
            for j in range(y - 1, y + 2):
                if (not(i == x and j == y) and (i >= 0 and i < mapdata.info.width and j >= 0 and j < mapdata.info.height)):
                    index = grid_to_index(mapdata, i, j)
                    if (mapdata.data[index] == -1):
                        isTrue == True
                        return True
                        break
    
    return isTrue


def thresholdMap(mapdata):

    threshold_map = mapdata
    threshold_map.data = list(mapdata.data)

    for i in range(len(mapdata.data)):
        if (mapdata.data[i] >= 50):
            threshold_map.data[i] = 100
        elif (mapdata.data[i] < 50 and mapdata.data[i] >= 0):
            threshold_map.data[i] = 0
    
    return threshold_map

def gridList_to_pointList(mapdata, gridList):
    pointList = []

    for elem in gridList:
        pointList.append(grid_to_world(mapdata, elem[0], elem[1]))

    return pointList


def PoseStamped_to_GridCells(mapdata, pStamped):
    gridcell = GridCells()
    gridcell.header.frame_id = 'map'
    gridcell.cell_width = mapdata.info.resolution
    gridcell.cell_height = mapdata.info.resolutionf

    worldPoint = PoseStamped_to_WorldPoint(pStamped)
    gridcell.cells = worldPoint

    return gridcell


def Grid_to_PoseStamped(mapdata, grid):
    pose = PoseStamped()
    grid_x = grid[0]
    grid_y = grid[1]

    worldPoint = grid_to_world(mapdata, grid_x, grid_y)
    
    pose.pose.position.x = worldPoint.x
    pose.pose.position.y = worldPoint.y

    return pose


def PoseStamped_to_GridCoor(mapdata, pStamped):
    worldPoint = PoseStamped_to_WorldPoint(pStamped)

    grid_cell = world_to_grid(mapdata, worldPoint)

    return grid_cell


def PoseStamped_to_WorldPoint(pStamped):
    worldPoint = Point()
    worldPoint.x = pStamped.pose.position.x
    worldPoint.y = pStamped.pose.position.y

    return worldPoint
