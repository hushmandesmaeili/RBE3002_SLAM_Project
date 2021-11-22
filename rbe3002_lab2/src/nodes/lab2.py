#!/usr/bin/env python2

import rospy
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0

        self.path = []

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
        # rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.follow_path)

        rospy.Subscriber('/robot_path', Path, self.update_path)
        #pass # delete this when you implement your code

        # Give ROS time to initial nodes
        rospy.sleep(2)

    def update_path(self, msg):
        self.path = msg.poses

    def follow_path(self):
        for pose in self.path:
            self.go_to(pose)

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        msg_cmd_vel = Twist()

        # Linear velocity
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        ### Publish the message
        self.speed_pub.publish(msg_cmd_vel)

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        px_0 = self.px
        py_0 = self.py
        pth_0 = self.pth

        aspeed = 0.1
        kp_th = 10

        px_goal = px_0 + distance*math.cos(pth_0)
        py_goal = py_0 + distance*math.sin(pth_0)
        # print(px_goal)
        # print(py_goal)

        heading_goal = math.atan2(py_goal - py_0, px_goal - px_0)

        distance_error = (math.sqrt((px_goal - self.px)**2 + (py_goal- self.py)**2)) 

        TOLERANCE = 0.1

        self.send_speed(linear_speed, 0)

        while ((distance_error > TOLERANCE)):
            distance_error = (math.sqrt((px_goal - self.px)**2 + (py_goal- self.py)**2))
            heading_error = heading_goal - self.pth
            self.send_speed(linear_speed, aspeed * heading_error * kp_th)
            print(distance_error)
            rospy.sleep(0.05)

        self.send_speed(0, 0)

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        ### REQUIRED CREDIT
        pth_0 = self.pth

        TOLERANCE = 0.007

        desired_angle = pth_0 + angle
        error = self.computeAngleError(pth_0, desired_angle)

        if (error < 0):
            aspeed = -aspeed

        self.send_speed(0, aspeed)

        while (abs(error) > (TOLERANCE)):
            error = self.computeAngleError(self.pth, desired_angle)
            # print(error)
            rospy.sleep(0.05)

        self.send_speed(0, 0)

    def computeAngleError(self, ang1, ang2):

        while (ang1 < 0):
            ang1 += 2*math.pi

        while (ang1 > 2*math.pi):
            ang1 -= 2*math.pi

        while (ang2 < 0):
            ang2 += 2*math.pi

        while (ang2 > 2*math.pi):
            ang2 -= 2*math.pi

        error = ang2 - ang1

        while (error < -math.pi):
            error += 2*math.pi

        while (error > math.pi):
            error -= 2*math.pi

        return (error)

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT

        ### MATH FOR X AND Y DISTANCE
        px_0 = self.px
        py_0 = self.py
        px_goal = msg.pose.position.x
        py_goal = msg.pose.position.y
        # print(px_goal, py_goal)

        ### MATH FOR THETA DISTANCE FOR ROTATION 1
        pth_0 = self.pth
        pth_goal_1 = math.atan2(py_goal - py_0, px_goal - px_0)

        # CODE FOR FIRST ROTATION
        angle_distance = pth_goal_1 - pth_0
        self.rotate(angle_distance, 0.15)
        rospy.sleep(1)

        # CODE FOR DRIVING TO GOAL
        linear_distance = math.sqrt((px_goal - self.px)**2 + (py_goal- self.py)**2)
        print(linear_distance)
        self.drive(linear_distance, 0.2)
        rospy.sleep(1)

        # CODE FOR SECOND ROTATION
        ### MATH FOR THETA DISTANCE FOR ROTATION 2
        pth_0 = self.pth
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        pth_goal_2 = yaw
        pth_curr = pth_goal_2 - pth_0
        self.rotate(pth_curr, 0.15)    # CHANGE TO DEPEND ON LOCATION OF FINAL ANGLE
        rospy.sleep(1)

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

    # def arc_to(self, msg):    
    #     """
    #     Drives to a given position in an arc.
    #     :param msg [PoseStamped] The target pose.
    #     """
    #     ### EXTRA CREDIT
        
    #     ## Constants and variables defined
    #     LSPEED = 0.2
    #     ASPEED = 0.7
        
    #     # Kp value for linear speed
    #     kp_lspeed = 0.5
        
    #     # Ki value for linear speed, 
    #     # sum of error variable for integral term plus errorBound
    #     ki_lspeed = 0.035
    #     sum_error = 0
    #     errorBound = 10

    #     # Kp value for heading
    #     kp_th = 0.7
        
    #     ### Initial and goal x,y positions
    #     px_0 = self.px
    #     py_0 = self.py
    #     px_goal = msg.pose.position.x
    #     py_goal = msg.pose.position.y

    #     ### Initial orientation
    #     pth_0 = self.pth
        
    #     TOLERANCE = 0.02
        
    #     while (abs(self.px - px_goal) > TOLERANCE or abs(self.py - py_goal) > TOLERANCE):
    #         pth_goal = math.atan2(py_goal - self.py, px_goal - self.px)
            
    #         distance_error = (math.sqrt((px_goal - self.px)**2 + (py_goal- self.py)**2))
    #         heading_error = self.computeAngleError(self.pth, pth_goal)
            
    #         sum_error = sum_error + distance_error
    #         if (sum_error > errorBound):
    #             sum_error = errorBound
    #         elif (sum_error < -1*errorBound):
    #             sum_error = -1*errorBound

    #         lspeed = LSPEED * (kp_lspeed * distance_error + ki_lspeed * sum_error)
    #         if (lspeed < 0.05):
    #             lspeed = 0.05
    #         elif (lspeed > LSPEED):
    #             lspeed = LSPEED

    #         aspeed = ASPEED * heading_error * kp_th
            
    #         self.send_speed(lspeed, aspeed)
    #         # print(aspeed)
    #         # print(lspeed)

    #         rospy.sleep(0.05)
        
    #     self.send_speed(0, 0)
    #     rospy.sleep(1)
            

    #     # CODE FOR SECOND ROTATION
    #     ### MATH FOR THETA DISTANCE FOR ROTATION 2
    #     pth_0 = self.pth
    #     quat_orig = msg.pose.orientation
    #     quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
    #     (roll, pitch, yaw) = euler_from_quaternion(quat_list)
    #     pth_goal_2 = yaw
    #     pth_curr = pth_goal_2 - pth_0
    #     self.rotate(pth_curr, 0.3)    # CHANGE TO DEPEND ON LOCATION OF FINAL ANGLE
    #     rospy.sleep(1)
    #     # pass # delete this when you implement your code

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code

    def run(self):
        #self.send_speed(0.2, 0)
        # self.drive(2, 0.2)
        #self.rotate(1.57, 0.3)
        rospy.spin()


if __name__ == '__main__':
    Lab2().run()
