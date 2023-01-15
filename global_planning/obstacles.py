import shapely
import shapely.geometry
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
class ObstacleRectangle:
    '''
    Class for rectangular obstacles used in RRT star planning
    Input:  x1, y1: bottom left corner of obstacle
            x2, y2: top right corner of obstacle
    '''
    def __init__(self, x1, y1, x2, y2, margin=0.4):
        self.type = 'rectangle'
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = x2 - x1
        self.height = y2 - y1
        self.object = shapely.geometry.box(x1-margin, y1-margin, x2+margin, y2+margin)

class ObstacleCircle:
    '''
    Class for circular obstacles used in RRT star planning
    Input:  x, y: center of obstacle
            radius: radius of obstacle
    '''
    def __init__(self, x, y, radius, margin=0.4):
        self.type = 'circle'
        self.x = x
        self.y = y
        self.radius = radius
        self.object = shapely.geometry.Point(x, y).buffer(radius+margin)

class Plotter:
    '''
    Class to plot the RRT star tree and path
    '''
    def __init__(self, start, end, obstacleList):
        '''
        Initialize the class variables
        Input:  start: start node
                end: goal node
                obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
        Output: None
        '''
        self.start = start
        self.end = end
        self.obstacleList = obstacleList
        self.nodeList = []


    def plotFinalTree(self, finalNode, path):
        '''
        Plot the final tree
        Input:  nodeList: list of all nodes
        Output: None
        '''
        plt.figure(figsize=(9,9))
        for node in self.nodeList:
            plt.plot(node.x,node.y, color = 'darkgrey', marker = 'o', markersize = 5)
            if node.parent:
                plt.plot((node.parent.x,node.x), (node.parent.y,node.y), color='darkgrey', linestyle='-')

        for obs in self.obstacleList:
            if obs.type == 'rectangle':
                plt.gca().add_patch(plt.Rectangle((obs.x1,obs.y1),obs.width,obs.height, fc = 'darkred', ec='darkred'))
            elif obs.type == 'circle':
                plt.gca().add_patch(plt.Circle((obs.x,obs.y),obs.radius, fc = 'darkred', ec='darkred'))

        plt.plot([self.start.x], self.start.y, color = 'green', marker = 's', markersize = 20)
        plt.plot([self.end.x],self.end.y, color = 'green', marker = '*', markersize = 20)

        for node in path:
            if node.parent:
                plt.plot((node.parent.x,node.x), (node.parent.y,node.y), color='dodgerblue', linestyle='-')
            plt.plot(node.x,node.y, color = 'orange', marker = 'o', markersize = 6)

        plt.figtext(0.5, 0.01, 'cost (m) = ' + str(round(self.cost(finalNode),2)) + ' time (s) = ' +str(round(self.totalTime, 2)) + ' nodes: ' + str(len(self.nodeList)), wrap=True, horizontalalignment='center', fontsize=12)
        plt.axis([self.minRand, self.maxRand, self.minRand, self.maxRand])
        plt.legend
        redBox = Patch(color='darkred', label='Obstacle')
        bluePath = Patch(color='dodgerblue', label='Edges goalpath')
        orangeNodes = Patch(color='orange', label='Vertices goalpath')
        greyNodes = Patch(color='darkgrey', label='Tree')
        greenStart = Patch(color='green', label='Start/Goal')
        handles = [redBox, bluePath, orangeNodes, greyNodes, greenStart]
        plt.legend(handles=handles, bbox_to_anchor=(1,1), borderaxespad=0.)
        plt.tight_layout()
        plt.grid(False)
        plt.pause(0.01)