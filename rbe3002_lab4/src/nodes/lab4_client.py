# import math
# import time
# import rospy
# from nav_msgs.srv import GetPlan, GetMap
# from nav_msgs.msg import GridCells, OccupancyGrid, Path
# from geometry_msgs.msg import Point, Pose, PoseStamped
# from priority_queue import PriorityQueue
# import map_functions as mapf

# class Lab4Client:

#     def __init__(self):
#         """
#         Class constructor
#         """


    

#     def navigation_client(self, goal):

#         # ### RETURNS POSESTAMPED OBJECT
#         # goal = msg

#         rospy.loginfo("Requesting the path")
#         rospy.wait_for_service('navigate_to')
#         go_to = rospy.ServiceProxy('navigate_to', PoseStamped)
#         try:
#             resp1 = go_to(goal)
#         except rospy.ServiceException as exc:
#             print("Service did not process request: " + str(exc))

#     def run(self):


#         rospy.spin()


# if __name__ == '__main__':
#     Lab4Client().run()



