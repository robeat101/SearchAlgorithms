#!/usr/bin/env python

"""
Author: Rohit Bansal
Description: Read a map, get a start and goal position, and perform Astar search. 
            Publish list of searched cells and frontier cells. 
"""
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid, GridCells

class Astar: 

    def beginSearch(self):
        self.updateStartLocation()
        self.updateGoal()



    def defineMap(self, map): 
        self.isMapDefined = True
        self.map = map
        return self.testSearchConditions()
        
    def setRobotLocation(self, robotLocation): 
        self.isRobotLocationSet = True
        self.robotLocation = robotLocation
        return self.testSearchConditions()

    def updateStartLocation(self): 
        self.startLocationCells = self.newGridCell()
        startLoc = Point(); 
        startLoc.x = self.robotLocation.pose.pose.position.x
        startLoc.y = self.robotLocation.pose.pose.position.y
        startLoc.z = 0
        self.startLocationCells.cells.append(startLoc)
        self.pub_start.publish(self.startLocationCells)
    
    def updateGoal(self):
        self.goalCells = self.newGridCell()
        goalLoc = Point(); 
        goalLoc.x = self.navGoal.pose.position.x
        goalLoc.y = self.navGoal.pose.position.y
        goalLoc.z = 0
        self.goalCells.cells.append(goalLoc)
        self.pub_goal.publish(self.goalCells)

    def newGridCell(self): 
        gridcell = GridCells()
        gridcell.cell_height = self.robotResolution
        gridcell.cell_width = self.robotResolution
        gridcell.header.frame_id = 'map'
        return gridcell

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

        #Setup publishers
        self.pub_start = rospy.Publisher('/robot/start', GridCells, latch=True, queue_size=1)
        self.pub_goal = rospy.Publisher('/robot/goal', GridCells, latch=True, queue_size=1)


        




if __name__ == '__main__': 

    #Create an Astar object
    Astar = Astar()
    rospy.spin()