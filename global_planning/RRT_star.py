"""
Author: Willem Momma 
Path planning using RRT star algorithm
"""
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from global_planning.obstacles import ObstacleRectangle, Plotter
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
        self.parent = None
        self.cost = 0
class RRT_star:
    '''
    Class for RRT star planning
    Goal: find optimal path from start to goal avoiding obstacles
    Returns: path as a list of nodes [[x1, y1], [x2, y2], ...]
    Also plots the growing tree, final path and obstacles
    '''

    def __init__(self, start, goal, obstacleList, randArea,
                 maxIter=2000,
                 probGoal=0.05,
                 threshold=1,
                 maxExpansion=10,
                 searchGamma=60
                 ):
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
        self.totalTime = 0
        self.obstacleList = obstacleList

        # behavior settings for RRT
        self.maxIter = maxIter  # maximum number of iterations
        self.probGoal = probGoal  # probability to sample goal
        self.threshold = threshold  # radius of accepted area within goal
        self.maxExpansion = maxExpansion  # max distance to expand each collision free step
        self.searchGamma = searchGamma # gamma parameter for RRT* optimilization search radius
        self.goalReached = False

    def planning(self):
        '''
        Path planning using RRT star algorithm
        Uses class methods to generate a path from start to goal
        Samples random points, finds nearest node, steers towards random point, checks for collision
        If no collision, finds neighbors in radius neighbor_radius, chooses parent, rewire tree, add node to node list
        Output: path as a list of nodes [[x1, y1], [x2, y2], ...]
        '''
        start = time.time()
        self.nodeList = [self.start]
        for i in range(self.maxIter):
            qRand = self.getRandomPoint()
            if qRand == None:
                continue
            nearestNode = self.getNearestNode(self.nodeList, qRand)
            newNode = self.steeringFunction(nearestNode, qRand)
            if self.lineCollisionCheck(newNode, nearestNode):
                neighbors = self.findNeighbors(self.nodeList, newNode)
                self.nodeList.append(newNode)
            # Choose best parent
            else:
                continue
            for neighbor in neighbors:
                if self.lineCollisionCheck(neighbor, newNode) \
                        and self.cost(neighbor) + self.euclideanDistance(newNode, neighbor) < self.cost(newNode):
                    newNode.parent = neighbor
            # Rewire
            for neighbor in neighbors:
                if self.lineCollisionCheck(newNode, neighbor):
                    if self.cost(newNode) + self.euclideanDistance(newNode, neighbor) < self.cost(neighbor):
                        neighbor.parent = newNode
        end = time.time()
        self.totalTime = end - start
        print("Time taken: ", self.totalTime)

        finalNode = self.getNearestNode(self.nodeList, self.end)
        if self.euclideanDistance(finalNode, self.end) < self.threshold:
            print("Goal Reached!")
            self.goalReached = True
        else:
            print("Goal not reached, try increasing maxIter and/or maxExpansion")

        path = self.finalPath(finalNode)
        #Plotter.plotFinalTree(self, finalNode, path)
        #plt.show()
        return path

    def results(self):
        '''
        Returns the final path, total time taken and number of iterations
        '''
        finalNode = self.getNearestNode(self.nodeList, self.end)
        path = self.finalPath(finalNode)
        return path, self.totalTime, self.maxIter

    def findNeighbors(self, allNodes, newNode):
        '''
        Find neighbors of the new node within a radius of searchRadius
        Input:  allNodes: list of all nodes
                newNode: node to be compared with all nodes
                Output: neighborList: list of nodes within searchRadius of newNode
        '''
        euclideanDistances = [self.euclideanDistance(node, newNode) for node in allNodes]
        neighborList = []
        searchRadius = self.searchGamma * (math.log(len(allNodes)) / len(allNodes)) ** (1/3)
        #searchRadius = 10
        for i in range(len(euclideanDistances)):
            if euclideanDistances[i] < searchRadius:
                neighborList.append(allNodes[i])
            else:
                continue
        return neighborList

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
        return math.dist([node1.x, node1.y], [node2.x, node2.y])

    def getNearestNode(self, allNodes, newNode):
        '''
        Find the nearest node from all existing nodes
        Input:  allNodes: list of all nodes
                newNode: node to be compared with all nodes
        Output: nearestNode: node in allNodes closest to newNode
        '''
        euclideanDistances = [self.euclideanDistance(
            node, newNode) for node in allNodes]
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
        newNode.parent = fromNode
        return newNode

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

    def finalPath(self, finalNode):
        '''
        Generate final path from start to goal
        Input:  nodeList: list of all nodes
                goal: goal node
        Output: path: list of nodes from start to goal
        '''
        path = []
        node = finalNode
        while node.parent:
            path.append(node)
            node = node.parent
        path.append(self.start)
        return path

def main(obstacles, start, goal_position, margin):
    '''
    Main function
    Specify start, goal, obstacles, and random area

    optional settings:
        maxIter: maximum number of iterations, default 1200
        maxExpansion: maximum distance to expand tree, default 1
        r: radius of circle around node to check for collision, default 1
        probGoal: probability of selecting goal as random node, default 0.05
        threshold: threshold to check if goal is reached, default 0.5
        searchGamma: gamma value for RRT*, default 40

    output: path from start to goal, interpolated path
    '''
    obstacleList = []
    for i in range(len(obstacles)):
        obstacle = ObstacleRectangle(
            obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3], margin)
        obstacleList.append(obstacle)

    start = start
    goal = goal_position
    randArea = [-10, 10]
    rrt = RRT_star(start, goal, obstacleList, randArea)
    path = rrt.planning()
    trajectory = []
    resolution = 0.025
    for node in path:
        # fill spaces in between nodes with linear interpolation
        if node.parent:
            dist = np.sqrt((node.x - node.parent.x)**2 +
                           (node.y - node.parent.y)**2)
            n = int(dist / resolution)
            x = np.linspace(node.x, node.parent.x, n)[1:]
            y = np.linspace(node.y, node.parent.y, n)[1:]
            for i in range(len(x)):
                trajectory.append(np.array([x[i], y[i]]))
    return trajectory[::-1]