
import random 
import math
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString
from matplotlib.patches import Circle

class obstacleSquare:
    '''
    Class for squared obstacles used in RRT star planning
    Input:  x, y: center of obstacle
            size: length of side of obstacle
    '''
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
        self.x1 = x - size/2
        self.x2 = x + size/2
        self.y1 = y - size/2
        self.y2 = y + size/2
class RRT_star:
    ''' 
    Class for RRT star planning 
    Goal: find optimal path from start to goal avoiding obstacles
    Returns: path as a list of nodes [[x1, y1], [x2, y2], ...]
    Also plots the growing tree, final path and obstacles
    '''
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
            self.cost = 0

    def __init__(self, start, goal, obstacleList, randArea):
        '''
        Initialize the class variables
        Input:  start: start node
                goal: goal node
                obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
                randArea: random sampling area equal to map size
                maxIter: maximum number of expansions
        Output: None
        '''
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1]) 
        self.minRand = randArea[0]
        self.maxRand = randArea[1]
        self.obstacleList = obstacleList
    
        #behavior settings for RRT
        self.maxIter = 200 #maximum number of iterations
        self.probGoal = 0.0 #probability to sample goal 
        self.threshold = 2 #radius of accepted area within goal
        self.maxExpansion = 5 #max distance to expand each collision free step
        self.searchRadius = 5 #radius to find nearest neighbors for RRT* optimilization 
    
    def planning(self):
        '''
        Path planning using RRT star algorithm
        Uses class methods to generate a path from start to goal
        Samples random points, finds nearest node, steers towards random point, checks for collision
        If no collision, finds neighbors in radius neighbor_radius, chooses parent, rewire tree, add node to node list
        Output: path as a list of nodes [[x1, y1], [x2, y2], ...]
        '''
        self.nodeList = [self.start] 
        for i in range(self.maxIter):
            while self.goalCheck(self.nodeList[-1]):    
                qRand = self.getRandomPoint()
                if qRand == None:
                    continue
                nearestNode = self.getNearestNode(self.nodeList, qRand) 
                newNode = self.steeringFunction(nearestNode, qRand) 
                neighborList = self.findNeighbors(self.nodeList, newNode)
                newNode = self.optimizeTree(newNode,neighborList)
                if self.lineCollisionCheck(newNode, self.obstacleList):
                    self.nodeList.append(newNode)
                self.plotGraph(self.nodeList)
        finalNode = self.nodeList[-1]
        path = self.finalPath(finalNode)
        self.plotFinalPath(path)
        plt.show()
        return path

    def findNeighbors(self, allNodes, newNode):
        '''
        Find neighbors of the new node within a radius of searchRadius
        Input:  allNodes: list of all nodes
                newNode: node to be compared with all nodes
                Output: neighborList: list of nodes within searchRadius of newNode
        '''
        euclideanDistances = [self.euclideanDistance(node, newNode) for node in allNodes]
        neighborList = []
        for i in range(len(euclideanDistances)):
            if euclideanDistances[i] < self.searchRadius:
                neighborList.append(allNodes[i])
            else:
                continue
        return neighborList


    def optimizeTree(self, newNode, neighborList):
        '''
        First connects newNode to neighbors, then checks if there is a better parent for each neighbor
        If there is a better parent, rewire the tree
        Input:  newNode: node to be connected to neighbors
                neighborList: list of nodes within searchRadius of newNode
        Output: None
        '''
        for neighbor in neighborList:
            placer = self.steeringFunction(neighbor, newNode) #connect all neighbors to newNode
            if self.lineCollisionCheck(placer, self.obstacleList): #check for collision
                if placer.cost < newNode.cost: #if there is a better parent, rewire the tree
                    newNode.parent = neighbor
                    newNode.path_x = [newNode.x, newNode.parent.x]
                    newNode.path_y = [newNode.y, newNode.parent.y]
                    newNode.cost = newNode.cost + self.euclideanDistance(newNode, newNode.parent)

        for neighbor in neighborList:
            placer = self.steeringFunction(newNode, neighbor)
            if self.lineCollisionCheck(placer, self.obstacleList):
                if placer.cost < neighbor.cost:
                    neighbor.parent = newNode
                    neighbor.path_x = [neighbor.x, neighbor.parent.x]
                    neighbor.path_y = [neighbor.y, neighbor.parent.y]
                    neighbor.cost = neighbor.cost + self.euclideanDistance(neighbor, neighbor.parent)
        return newNode

            
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
                if randomx >= obs.x1 and randomx <= obs.x2 and randomy >= obs.y1 and randomy <= obs.y2:
                    return None
            qRand = self.Node(randomx, randomy)
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
            newNode = self.Node(fromNode.x, fromNode.y)
            newNode.x += self.maxExpansion * math.cos(theta)
            newNode.y += self.maxExpansion * math.sin(theta)
        else:    
            newNode = self.Node(toNode.x, toNode.y)
        newNode.path_x = [fromNode.x, newNode.x]
        newNode.path_y = [fromNode.y, newNode.y]
        newNode.parent = fromNode
        newNode.cost = fromNode.cost + self.euclideanDistance(fromNode, newNode)
        return newNode
    
    def goalCheck(self, node):
        '''
        Check if goal is reached within specified radius of endpoint 
        Input: node: latest generated node 
        Output: boolean True if goal reached 
        '''
        if ((node.x - self.end.x)**2 + (node.y - self.end.y)**2 >= self.threshold**2):
            return True
    
    def lineCollisionCheck(self, newNode, obstacleList):
        '''
        Check if line between two nodes collides with any obstacle
        Input:  node1: start node
                node2: end node
                obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
        Output: True if no collision, False if collision
        '''
        #create line object 
        line = LineString([(newNode.path_x[0], newNode.path_y[0]), (newNode.path_x[1], newNode.path_y[1])])
        for obs in obstacleList:
            polygon = Polygon([(obs.x1, obs.y1), (obs.x2, obs.y1), (obs.x2, obs.y2), (obs.x1, obs.y2)])
            if polygon.intersects(line):
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
        trajectory = []
        node = finalNode
        while node.parent:
            path.append(node)
            trajectory.append(np.array([node.x, node.y]))
            node = node.parent
        path.append(self.start)
        trajectory.append(np.array([self.start.x, self.start.y]))
        return path, trajectory
        
    def plotGraph(self, nodeList):
        '''
        Plot the graph
        Input:  nodeList: list of all nodes
                path: list of nodes from start to goal
        Output: None
        '''
        plt.clf()
        plt.plot([self.start.x], self.start.y, 'ro')
        plt.plot([self.end.x],self.end.y, 'go')

        for node in nodeList:
            plt.plot(node.x,node.y, 'yo')

            if node.parent:
                plt.plot(node.path_x, node.path_y, 'y-')

        for obs in self.obstacleList:
            plt.gca().add_patch(plt.Rectangle((obs.x1,obs.y1),obs.size,obs.size, fc = 'blue', ec='red'))
        
        plt.axis([self.minRand, self.maxRand, self.minRand, self.maxRand])
        plt.figtext(0.5, 0.01, 'RRT*, cost =' + str(round(nodeList[-1].cost,1)), wrap=True, horizontalalignment='center', fontsize=12)
        plt.grid(True)
        plt.pause(0.01)
    
    def plotFinalPath(self, path):
        '''
        Plot the final path
        Input:  path: list of nodes from start to goal
        Output: None
        '''
        plt.plot([self.start.x], self.start.y, 'ro')
        plt.plot([self.end.x],self.end.y, 'go')

        for node in path:
            plt.plot(node.x,node.y, 'yo')
            if node.parent:
                plt.plot(node.path_x, node.path_y, 'r-')
        
        for obs in self.obstacleList:
            plt.gca().add_patch(plt.Rectangle((obs.x1,obs.y1),obs.size,obs.size, fc = 'blue', ec='red'))
        
        plt.axis([self.minRand, self.maxRand, self.minRand, self.maxRand])
        plt.grid(True)
        plt.pause(0.01)

def main(): 
    '''
    Main function
    Specify start, goal, obstacles, and random area
    trajectorty: list of arrays with x, y coordinates of trajectory
    '''
    obstacle1 = obstacleSquare(12,5,5)
    obstacle2 = obstacleSquare(20,20,10)
    obstacle3 = obstacleSquare(32,40,8)
    obstacle4 = obstacleSquare(35,10,7)
    obstacleList = [obstacle1, obstacle2, obstacle3, obstacle4]
    start = [2,2]
    goal = [45, 43]
    randArea = [0,50]
    rrt = RRT_star(start, goal, obstacleList, randArea)
    path = rrt.planning()
    for node in path: 
        print(node.x,node.y)
if __name__ == '__main__':
    main()



