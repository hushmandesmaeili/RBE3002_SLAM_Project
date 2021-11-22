#!/usr/bin/env python

import math
import time
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from priority_queue import PriorityQueue
from tf.transformations import euler_from_quaternion

class PathPlannerClient:

    def __init__(self):
        
        """
        Class constructor
        """

        rospy.init_node('path_planner_client')

        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0

        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        #rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.plan_path_client)

        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        #pass # delete this when you implement your code

        # Give ROS time to initial nodes
        rospy.sleep(2)


    def plan_path_client(self, msg):

        ### RETURNS PATH OBJECT
        start = PoseStamped()
        start.pose.position.x = self.x
        start.pose.position.y = self.y
        start.pose.orientation = quaternion_from_euler(0, 0, self.pth)
        goal = msg
        tolerance = 0.1
        rospy.loginfo("Requesting the path")
        rospy.wait_for_service('plan_path')
        try:
            pathdata = rospy.ServiceProxy('plan_path', GetPlan)
            resp1 = pathdata(start, goal, tolerance)
            self.path_pub.publish(resp1.plan)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        # print(resp1.map)
        # return resp1.plan

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
        #self.send_speed(0.2, 0)
        # self.drive(2, 0.2)
        #self.rotate(1.57, 0.3)
        rospy.spin()


if __name__ == '__main__':
    Lab2().run()
    