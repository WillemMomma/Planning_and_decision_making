"""
Author: Willem Momma
RRT planning algorith 
"""

import random 
import math
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from global_planning.obstacles import Plotter 
import time 

class Node:
    '''
    Class to define the Nodes (i.e. vertices)
    Also specifies parent Node and path from parent to child 
    Input:  x, y: node coordinates
    '''
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None
class RRT:
    ''' 
    Class for RRT planning 
    Goal: find path from start to goal avoiding obstacles
    Input:  start: start node
            goal: goal node
            obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
            randArea: random sampling area equal to map size
            maxIter: maximum number of expansions
    '''

    def __init__(self, start, goal, obstacleList, randArea, maxIter, maxExpansion):
        '''
        Initialize the class variables
        Input:  start: start node
                goal: goal node
                obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
                randArea: random sampling area equal to map size
                maxIter: maximum number of expansions
        Output: None
        '''
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1]) 
        self.minRand = randArea[0]
        self.maxRand = randArea[1]
        self.obstacleList = obstacleList
        self.totalTime = 0
        #behavior settings for RRT
        self.probGoal = 0.005 #probability to sample goal 
        self.maxIter = maxIter #maximum number of iterations
        self.threshold = 0.5 #radius of accepted area within goal
        self.maxExpansion = maxExpansion #max distance to expand each collision free step
        
    def planning(self):
        '''
        Path planning using RRT star algorithm
        Uses class methods to generate a path from start to goal
        Samples random points, finds nearest node, steers towards random point, checks for collision
        If no collision, finds neighbors in radius neighbor_radius, chooses parent, rewire tree, add node to node list
        Output: path as a list of nodes [[x1, y1], [x2, y2], ...]
        '''
        startTime = time.time()
        self.nodeList = [self.start] 
        for i in range(self.maxIter):
            while self.goalCheck(self.nodeList[-1]):    
                qRand = self.getRandomPoint()
                if qRand == None:
                    continue
                nearestNode = self.getNearestNode(self.nodeList, qRand) 
                newNode = self.steeringFunction(nearestNode, qRand) 
                if self.lineCollisionCheck(newNode, nearestNode):
                    self.nodeList.append(newNode)
        path = self.finalPath()
        finalNode = self.nodeList[-1]
        endTime = time.time()
        self.totalTime = endTime - startTime
        # Plotter.plotFinalTree(self, finalNode, path)
        # plt.show()
        return path
    
    def cost(self, node):
        '''
        Cost of a node is the sum of the euclidean distances from the start node to the input node
        Input: node: node whose cost is to be calculated
        Output: cost: cost of the node
        '''
        cost = 0
        while node.parent:
            cost += self.euclideanDistance(node, node.parent)
            node = node.parent
        return cost

    def getRandomPoint(self):
        '''
        Samples random node from the collision free configuration space
        Input: None
        Output: q_rand: random node
        '''
        if random.random() <= self.probGoal:
            qRand = self.end
            return qRand
        else:
            randomx = random.uniform(self.minRand, self.maxRand)
            randomy = random.uniform(self.minRand, self.maxRand)
            for obs in self.obstacleList:
                if obs.type == 'circle':
                    if (randomx - obs.x) ** 2 + (randomy - obs.y) ** 2 <= obs.radius ** 2:
                        return None
                elif obs.type == 'rectangle':
                    if randomx >= obs.x1 and randomx <= obs.x2 and randomy >= obs.y1 and randomy <= obs.y2:
                        return None
            qRand = Node(randomx, randomy)
            return qRand
    
    def euclideanDistance(self, node1, node2):
        '''
        Euclidean distance between two nodes, measure for distance in R^2
        Input: node1, node2
        Output: distance between node1 and node2
        '''
        return math.dist([node1.x,node1.y],[node2.x,node2.y])

    def getNearestNode(self, allNodes, newNode):
        '''
        Find the nearest node from all existing nodes
        Input:  allNodes: list of all nodes
                newNode: node to be compared with all nodes
        Output: nearestNode: node in allNodes closest to newNode
        '''
        euclideanDistances = [self.euclideanDistance(node, newNode) for node in allNodes]
        minDist = min(euclideanDistances)
        minDistanceIndex = euclideanDistances.index(minDist)
        nearestNode = allNodes[minDistanceIndex]
        return nearestNode
    
    def steeringFunction(self, fromNode, toNode):
        '''
        Simulates a unicycle steering towards a random point
        Input:  fromNode: node from which steering starts
                toNode: node towards which steering is done
        Output: newNode: node after steering, includes path from fromNode to newNode
        '''
        theta = math.atan2(toNode.y - fromNode.y, toNode.x - fromNode.x)
        dist = self.euclideanDistance(fromNode, toNode)
        if dist > self.maxExpansion:
            newNode = Node(fromNode.x, fromNode.y)
            newNode.x += self.maxExpansion * math.cos(theta)
            newNode.y += self.maxExpansion * math.sin(theta)
        else:    
            newNode = Node(toNode.x, toNode.y)
        newNode.path_x = [fromNode.x, newNode.x]
        newNode.path_y = [fromNode.y, newNode.y]
        newNode.parent = fromNode
        return newNode
    
    def goalCheck(self, node):
        '''
        Check if goal is reached within specified radius of endpoint 
        Input: node: latest generated node 
        Output: boolean True if goal reached 
        '''
        if ((node.x - self.end.x)**2 + (node.y - self.end.y)**2 >= self.threshold**2):
            return True
    
    def lineCollisionCheck(self, node1, node2):
        '''
        Check if line between two nodes collides with any obstacle
        Input:  node1: start node
                node2: end node
                obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
        Output: True if no collision, False if collision
        '''
        line = LineString([(node1.x, node1.y), (node2.x, node2.y)])
        for obs in self.obstacleList:
            if line.intersects(obs.object):
                return False
        return True

    def finalPath(self):
        '''
        Generate final path from start to goal
        Input:  nodeList: list of all nodes
                goal: goal node
        Output: path: list of nodes from start to goal
        '''
        path = []
        node = self.nodeList[-1]
        while node.parent:
            path.append(node)
            node = node.parent
        path.append(self.start)
        return path