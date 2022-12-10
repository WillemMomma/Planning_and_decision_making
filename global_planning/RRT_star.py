
import random 
import math
import matplotlib.pyplot as plt
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None
class obstacleSquare:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
        self.x1 = x - size/2
        self.x2 = x + size/2
        self.y1 = y - size/2
        self.y2 = y + size/2
class obstacleCircle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

class RRT_star:
    ''' 
    Class for RRT star planning 
    Goal: find a path from start to goal avoiding obstacles
    Returns: path as a list of nodes [[x1, y1], [x2, y2], ...]
    Also plots the growing tree, final path and obstacles
    '''
    def __init__(self, start, goal, obstacleList, randArea, maxIter=500, neighborRadius=50):
        '''
        Initialize the class variables
        Input: start: start node
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
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.neighborRadius = neighborRadius
        self.expandDistance = 30.0 #distance by which nodes are expanded towards random points

    def planning(self):
        '''
        Path planning using RRT star algorithm
        Uses class methods to generate a path from start to goal
        Samples random points, finds nearest node, steers towards random point, checks for collision
        If no collision, finds neighbors in radius neighbor_radius, chooses parent, rewire tree, add node to node list
        Output: path as a list of nodes [[x1, y1], [x2, y2], ...]
        '''
        self.nodeList = [self.start] # initialize node list with start node
        for i in range(self.maxIter):
            qRand = self.getRandomPoint() #sample random point
            nearestNode = self.getNearestNode(self.nodeList, qRand) #find nearest node from random point to existing nodes
            newNode = self.steeringFunction(nearestNode, qRand) #create new node by steering towards random point
            if self.checkCollision(newNode, self.obstacleList): #returns True if no collision
                neighbors = self.findNeighbors(newNode) #find neighbors within a radius, neighbor_radius
                nodeUpdatedParent = self.chooseParent(newNode, neighbors) #returns None if no parent is found, if parent is found, returns the node with updated parent
                if nodeUpdatedParent: #returns True if parent is found
                    self.rewire(nodeUpdatedParent, neighbors) #rewire the tree
                    self.nodeList.append(nodeUpdatedParent) #add the node to the node list
        path = self.generateFinalPath(self.nodeList, self.end) #generate final path
        return path

    def getRandomPoint(self):
        '''
        Samples random node within the random sampling area
        Input: None
        Output: q_rand: random node
        '''
        randomx = random.uniform(self.minRand, self.maxRand)
        randomy = random.uniform(self.minRand, self.maxRand)
        q_rand = self.Node(randomx, randomy)
        return q_rand
    
    def euclideanDistance(self, node1, node2):
        '''
        Euclidean distance between two nodes
        Input: node1, node2
        Output: distance between node1 and node2
        '''
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def getNearestNode(self, allNodes, newNode):
        '''
        Find the nearest node from all existing nodes
        Input:  allNodes: list of all nodes
                newNode: node to be compared with all nodes
        Output: nearestNode: node in allNodes closest to newNode
        '''
        euclideanDistances = [self.euclideanDistance(node, newNode) for node in allNodes]
        minDistanceIndex = euclideanDistances.index(min(euclideanDistances))
        nearestNode = allNodes[minDistanceIndex]
        return nearestNode
    
    def steeringFunction(self, fromNode, toNode):
        '''
        Simulates a unicycle steering towards a random point
        Input:  fromNode: node from which steering starts
                toNode: node towards which steering is done
        Output: newNode: node after steering, includes path from fromNode to newNode
        '''
        newNode = self.Node(fromNode.x, fromNode.y)
        newNode.path_x = [newNode.x]
        newNode.path_y = [newNode.y]
        theta = math.atan2(toNode.y - fromNode.y, toNode.x - fromNode.x) #angle between fromNode and toNode
        newNode.x += self.expandDistance * math.cos(theta) #
        newNode.y += self.expandDistance * math.sin(theta)
        newNode.path_x.append(newNode.x)
        newNode.path_y.append(newNode.y)
        newNode.parent = fromNode
        return newNode

    def checkCollision(node, obstacleList):
        '''
        Check if node collides with any obstacle
        Input:  node: node to be checked for collision
                obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
        Output: True if no collision, False if collision
        '''
        for (obstacle) in obstacleList:
            if isinstance(obstacle, obstacleSquare):
                if node.x >= obstacle.x1 and node.x <= obstacle.x2 and node.y >= obstacle.y1 and node.y <= obstacle.y2:
                    return False
            elif isinstance(obstacle, obstacleCircle):
                if math.sqrt((node.x - obstacle.x)**2 + (node.y - obstacle.y)**2) <= obstacle.radius:
                    return False
        return True

    def findNeighbors(self, newNode):
        '''
        Find all nodes within a radius, neighbor_radius
        Input:  node: node for which neighbors are to be found
                nodeList: list of all nodes
        Output: neighbors: list of all nodes within a radius, neighbor_radius
        '''
        neighbors = []
        for node in self.nodeList:
            if self.euclideanDistance(node, newNode) <= self.neighborRadius:
                neighbors.append(node)
        return neighbors
    
    def chooseParent(self, node, neighbors):
        '''
        Choose parent for a node, best parent is the one with minimum cost from start to node
        Input:  node: node
                neighbors: list of all nodes within a radius, neighbor_radius
        Output: node: node with updated parent
        '''
        if neighbors:
            costs = [self.euclideanDistance(node, neighbor) + neighbor.cost for neighbor in neighbors]
            minCostIndex = costs.index(min(costs))
            node.parent = neighbors[minCostIndex]
            node.cost = costs[minCostIndex]
            return node
        return None

    def rewire(self, node, neighbors):
        '''
        Rewire the tree, if a node has a better parent, change the parent
        Input:  node: node
                neighbors: list of all nodes within a radius, neighbor_radius
        Output: None
        '''
        for neighbor in neighbors:
            if self.euclideanDistance(node, neighbor) + node.cost < neighbor.cost:
                neighbor.parent = node
                neighbor.cost = self.euclideanDistance(node, neighbor) + node.cost

    def generateFinalPath(self, nodeList, goal):
        '''
        Generate final path from start to goal
        Input:  nodeList: list of all nodes
                goal: goal node
        Output: path: list of nodes from start to goal
        '''
        path = []
        node = goal
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(node)
        return path
        
    def plotGraph(self, nodeList, path):
        '''
        Plot the graph
        Input:  nodeList: list of all nodes
                path: list of nodes from start to goal
        Output: None
        '''
        plt.clf()
        for node in nodeList:
            if node.parent:
                plt.plot(node.path_x, node.path_y, '-g')
        for node in path:
            plt.plot(node.path_x, node.path_y, '-r')
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def plotObstacles(self, obstacleList):
        '''
        Plot the obstacles
        Input:  obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
        Output: None
        '''
        for (obstacle) in obstacleList:
            if isinstance(obstacle, obstacleSquare):
                plt.plot([obstacle.x1, obstacle.x1], [obstacle.y1, obstacle.y2], '-k')
                plt.plot([obstacle.x1, obstacle.x2], [obstacle.y2, obstacle.y2], '-k')
                plt.plot([obstacle.x2, obstacle.x2], [obstacle.y2, obstacle.y1], '-k')
                plt.plot([obstacle.x2, obstacle.x1], [obstacle.y1, obstacle.y1], '-k')
            elif isinstance(obstacle, obstacleCircle):
                circle = plt.Circle((obstacle.x, obstacle.y), obstacle.radius, color='k')
                plt.gcf().gca().add_artist(circle)
        plt.pause(0.01)
