
import random 
import math
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString



class obstacleSquare:
    '''
    Class for squared obstacles
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
        self.maxIter = 100
        self.obstacleList = obstacleList
        
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
            qRand = self.getRandomPoint()
            if qRand == None:
                continue
            else:
                nearestNode = self.getNearestNode(self.nodeList, qRand) #find nearest node from random point to existing nodes            
                newNode = self.steeringFunction(nearestNode, qRand) #create new node by steering towards random point
                if self.lineCollisionCheck(newNode, self.obstacleList):
                    self.nodeList.append(newNode)
                    self.plotGraph(self.nodeList)
        plt.show()
        path = self.nodeList
        return path

    def getRandomPoint(self):
        '''
        Samples random node from the collision free configuration space 
        Input: None
        Output: q_rand: random node
        '''
        randomx = random.uniform(self.minRand, self.maxRand)
        randomy = random.uniform(self.minRand, self.maxRand)
        for obs in self.obstacleList:
            if randomx >= obs.x1 and randomx <= obs.x2 and randomy >= obs.y1 and randomy <= obs.y2:
                return None
        qRand = self.Node(randomx, randomy)
        return qRand
    
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
        newNode = self.Node(toNode.x, toNode.y)
        newNode.path_x = [fromNode.x, toNode.x]
        newNode.path_y = [fromNode.y, toNode.y]
        newNode.parent = fromNode
        

        return newNode
    
    def lineCollisionCheck(self, newNode, obstacleList):
        '''
        Check if line between two nodes collides with any obstacle
        Input:  node1: start node
                node2: end node
                obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
        Output: True if no collision, False if collision
        '''
        #generate line object 
        line = LineString([(newNode.path_x[0], newNode.path_y[0]), (newNode.path_x[1], newNode.path_y[1])])
        for obs in obstacleList:
            polygon = Polygon([(obs.x1, obs.y1), (obs.x2, obs.y1), (obs.x2, obs.y2), (obs.x1, obs.y2)])
            if polygon.intersects(line):
                return False
        return True

    def findNeighbors(self, newNode):
        pass
    
    def chooseParent(self, node, neighbors):
        pass

    def rewire(self, node, neighbors):
        pass

    def generateFinalPath(self, nodeList, goal):
        pass

        
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
        plt.grid(True)
        plt.pause(0.01)

def main_rrt():
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
    return path
    

if __name__ == '__main__':
    main()



