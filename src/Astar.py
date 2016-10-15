#!/usr/bin/env python

"""
Author: Rohit Bansal
Description: Read a map, get a start and goal position, and perform Astar search. 
            Publish list of searched cells and frontier cells. 
"""

from geometry_msgs.msg import PoseWithCovarianceStamped

class Astar: 

    def beginSearch(self):
        
    def setRobotLocation(self): 

    def setNavGoal(self): 

    def testSearchConditions(self): 
        

    def __init__(self, robotResolution = 0.2): 
        #Initialize node
        rospy.init_node('rbansal_Astar')

        #Initialize variables 
        self.robotResolution = robotResolution
        self.isRobotLocationSet = False
        self.isNavGoalSet = False
        self.isMapDefined = False


        #Setup subscribers
        self.map = rospy.Subscriber('/map', OccupancyGrid, self.beginSerarch, queue_size=1)
        self.robotLocation = rospy.Subscriber('/initialpose', self.setRobotLocation, queue_size=1)
        self.navGoal = rospy.Subscriber('/move_base_simple/goal', self.setNavGoal, queue_size=1)





if __name__ == '__main__': 

    #Create an Astar object
    Astar = Astar()
    rospy.spin()