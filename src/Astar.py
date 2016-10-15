#!/usr/bin/env python

"""
Author: Rohit Bansal
Description: Read a map, get a start and goal position, and perform Astar search. 
            Publish list of searched cells and frontier cells. 
"""
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

class Astar: 

    def beginSearch(self):
        rospy.logwarn("Search has begun.")

    def defineMap(self, map): 
        self.isMapDefined = True
        self.map = map
        return self.testSearchConditions()
        
    def setRobotLocation(self, robotLocation): 
        self.isRobotLocationSet = True
        self.robotLocation = robotLocation
        return self.testSearchConditions()

    def setNavGoal(self, navGoal): 
        self.isNavGoalSet = True
        self.navGoal = navGoal
        return self.testSearchConditions()

    def testSearchConditions(self): 
        if(self.isMapDefined == True and self.isNavGoalSet == True and self.isRobotLocationSet == True):
            return self.beginSearch()
        return False            


    def __init__(self, robotResolution = 0.2): 
        #Initialize node
        rospy.init_node('rbansal_Astar')

        #Initialize variables 
        self.robotResolution = robotResolution
        self.isRobotLocationSet = False
        self.isNavGoalSet = False
        self.isMapDefined = False


        #Setup subscribers
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.defineMap, queue_size=1)
        self.sub_robotLocation = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.setRobotLocation, queue_size=1)
        self.sub_navGoal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.setNavGoal, queue_size=1)





if __name__ == '__main__': 

    #Create an Astar object
    Astar = Astar()
    rospy.spin()