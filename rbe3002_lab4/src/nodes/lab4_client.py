#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from rbe3002_lab4.srv import NavigateTo, FrontierReachable, GetFrontier
from priority_queue import PriorityQueue
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scripts.map_functions import *


class Lab4Client:

    # phase_state = 1
    # phase1_state = 0
    # phase2_state = 0
    # phase3_state = 0

    # # PHASES States
    # PHASE_1 = 1
    # PHASE_2 = 2
    # PHASE_3 = 3
    # EXIT_STATE = 4

    # # PHASE 1 States
    # GET_FRONTIER = 0
    # PLAN_PATH = 1
    # NAVIGATE_PATH = 2

    # # PHASE 2 States


    # # Phase 3 States



    def __init__(self):
        """
        Class constructor
        """
        self.px = 0
        self.py = 0
        self.pth = 0
        self.currentPoseStamped = PoseStamped()


        self.phase_state = 1
        self.phase1_state = 0
        self.phase2_state = 0
        self.phase3_state = 0

        # PHASES States
        self.PHASE_1 = 1
        self.PHASE_2 = 2
        self.PHASE_3 = 3
        self.EXIT_STATE = 4

        # PHASE 1 States
        self.GET_FRONTIER = 0
        self.PLAN_PATH = 1
        self.NAVIGATE_PATH = 2

        # PHASE 2 States


        # Phase 3 States



        self.old_frontier = PoseStamped()
        self.goal_frontier = PoseStamped()
        self.path = 0
        self.first_run = True
        self.count = 1

        # Create publisher
        # cspacePub = rospy.Publisher('/cspace_map', OccupancyGrid, queue_size=10)
        
        # Create subscribers
        # gmapSub = rospy.Subscriber('map', OccupancyGrid, gmapCallback)
        # infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
        # cspaceSub = rospy.Subscriber('map', OccupancyGrid, self.getCSpace)

        odomSub = rospy.Subscriber('/odom', Odometry, self.update_odometry)

        # Create services
        
        # Initialize node
        rospy.init_node("lab4_client")
        
        rospy.sleep(2.5)
        rospy.loginfo("lab4_client node ready")


    
    def state_machine(self):

        # Phase State Machine
        # print(self.phase1_state)

        if (self.phase_state == self.PHASE_1):

            if (self.phase1_state == self.CHECK_POSITION):
                print('checking position')
                cspace_node_response = self.check_position_client()
                position_goal = cspace_node_response[0]
                is_valid_pose = cspace_node_response[1]

                if not(is_valid_pose):
                    self.navigate_to(position_goal)

                else:
                    self.phase1_state = self.GET_FRONTIER


            if (self.phase1_state == self.GET_FRONTIER):
                print('Getting Frontier')
                frontier_node_response = self.get_frontier_client()
                # print(frontier_node_response)
                self.old_frontier = self.goal_frontier
                temp_frontier = frontier_node_response[0]
                distance_frontier = euclidean_distance(self.old_frontier.pose.position.x, 
                                                        self.old_frontier.pose.position.y, 
                                                        temp_frontier.pose.position.x, 
                                                        temp_frontier.pose.position.y)

                if (distance_frontier > 0.2):
                    self.goal_frontier = temp_frontier
                elif (self.first_run):
                    self.goal_frontier = temp_frontier
                    self.first_run = False

                frontiers_to_explore = frontier_node_response[1]

                # print('Goal frontier is ', goal_frontier)

                if (not frontiers_to_explore):
                    print('Phase 1 COMPLETED')
                    self.phase_state == self.PHASE_2
                else:
                    print('Completed Frontier')
                    print(self.goal_frontier)
                    self.phase1_state = self.PLAN_PATH


            ## FIND A WAY TO PLAN PATH AND NAVIGATE AT THE SAME TIME
            elif (self.phase1_state == self.PLAN_PATH):
                print('Planning path')
                # print(goal_frontier)
                self.path = self.plan_path_client(self.goal_frontier)
                print('Completed Plan Path')
                self.phase1_state = self.NAVIGATE_PATH

            elif (self.phase1_state == self.NAVIGATE_PATH):
                print(self.path)
                # if (len(self.path) == 2):
                #     self.navigation_client(self.path[1])
                #     self.phase1_state = self.GET_FRONTIER
                # else:
                
                self.navigation_client(self.path[1])
                self.phase1_state = self.GET_FRONTIER

                # print('Navigation starting')
                # i = 1
                # print("PATH", self.path)
                # while i < len(self.path):
                # # for i in range(len(self.path)):
                #     print(" i = ", i)
                #     # if i != 0:
                #     if (len(self.path) == 1):
                #         i = 0

                #     self.navigation_client(self.path[i])
                #     self.path = self.plan_path_client(self.goal_frontier)
                #     i = 1
                    # else:
                    #     i += 1


                # print('Completed Navigation')
                # self.phase1_state = self.GET_FRONTIER
                # rospy.spin()

        elif (self.phase_state == self.PHASE_2):
            print('Phase 2')


        # elif (phase_state == PHASE_3):

        # elif (phase_state == EXIT_STATE):
    

    def navigation_client(self, goal):

        # ### RETURNS POSESTAMPED OBJECT
        # goal = msg

        rospy.loginfo("Requesting navigation")
        rospy.wait_for_service('navigate_to')
        go_to = rospy.ServiceProxy('navigate_to', NavigateTo)
        try:
            resp1 = go_to(goal)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    
    def plan_path_client(self, goal):

        ### RETURNS PATH OBJECT
        start = PoseStamped()
        start.header.frame_id = "odom"
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        orientation = quaternion_from_euler(0, 0, self.pth)
        start.pose.orientation.x = orientation[0]
        start.pose.orientation.y = orientation[1]
        start.pose.orientation.z = orientation[2]
        start.pose.orientation.w = orientation[3]
        
        # goal = msg

        # print(start, goal)

        tolerance = 0.05
        rospy.loginfo("Requesting the path")
        rospy.wait_for_service('plan_path')
        pathdata = rospy.ServiceProxy('plan_path', GetPlan)
        try:
            resp1 = pathdata(start, goal, tolerance)
            # return resp1.plan.poses
            # self.path_pub.publish(resp1.plan)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        return resp1.plan.poses

    def get_frontier_client(self):

        # ### RETURNS POSESTAMPED OBJECT
        # goal = msg

        rospy.loginfo("Requesting the frontier")
        rospy.wait_for_service('get_frontier')
        getFrontier = rospy.ServiceProxy('get_frontier', GetFrontier)
        try:
            resp1 = getFrontier(self.currentPoseStamped)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        response = (resp1.pose, resp1.frontiers)

        # return resp1.pose
        return response


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

        # Storing current pose as PoseStamped message for getFrontier
        # Service Proxy which sends and receives PoseStamped messages
        self.currentPoseStamped.pose.position.x = msg.pose.pose.position.x
        self.currentPoseStamped.pose.position.y = msg.pose.pose.position.y
        # self.currentPoseStamped.pose.orientation = msg.pose.pose.orientation


    def run(self):

        while (True):

            self.state_machine()

            # rospy.spin_once()


if __name__ == '__main__':
    Lab4Client().run()



