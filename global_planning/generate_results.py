"""
Author: Willem Momma
This file is used to generate results and test different configurations of RRT and RRT star. 
Run the file

"""
from obstacles import ObstacleRectangle, ObstacleCircle
from RRT_star import RRT_star
from RRT import RRT 

def testEnvOne():
    '''
    Test environment 1
    Used to evaluate the performance of RRT star with different parameters
    '''
    randArea = [0,50]
    start = [40,1]
    goal = [47,47]
    obstacleList = []
    #lines for obstacles
    for x in range(5, 51, 10):
        obstacleList.append(ObstacleRectangle(x, 0, x+2, 20))
    for y in range(30, 51, 10):
        obstacleList.append(ObstacleRectangle(20, y, 50, y+2))
    #circles for obstacles
    obstacleList.append(ObstacleCircle(11, 36, 6))
    obstacleList.append(ObstacleRectangle(20, 46.5, 22, 50))
    return randArea, start, goal, obstacleList

def testEnvTwo():
    '''
    Test environment 2
    Used to compare implementation of group 21 and Atsushi Sakai
    '''
    start = [1, 1]
    goal = [50, 50]
    randArea = [0, 50]
    obstacle_list = [
    (5, 5, 3),
    (15, 15, 5),
    (25, 25, 5),
    (5, 15, 5),
    (5, 25, 5),
    (40, 40, 4),
    (30, 10, 7),
    (45, 30, 4),
    (36,28,3)
    ]  # [x,y,size(radius)]
    obstacleList = []
    for obs in obstacle_list:
        obstacleList.append(ObstacleCircle(obs[0], obs[1], obs[2]))
    return randArea, start, goal, obstacleList

def testRRTStar(maxIter, maxExpansion, searchGamma, env):
    '''
    Test different configurations of RRT star
    Runs the planner function of RRT_star class, will plot the result
    Input:  maxIter: maximum number of iterations
            maxExpansion: maximum distance to expand tree
            searchGamma: number of nodes to search for nearest neighbour
            env: test environment
    Output: None
    '''
    if env ==1:
        randArea, start, goal, obstacleList = testEnvOne()
    elif env ==2:
        randArea, start, goal, obstacleList = testEnvTwo()
    rrt = RRT_star(start, goal, obstacleList, randArea, maxIter=maxIter, maxExpansion=maxExpansion, probGoal=0.05, threshold=1, searchGamma=searchGamma)
    path = rrt.planning()
    print('Number of nodes in final tree: ', len(rrt.nodeList))
    print('Number of nodes in final path: ', len(path))
    return None

def testRRT(maxIter, maxExpansion, env):
    '''
    Test different configurations of RRT
    Runs the planner function of RRT class, will plot the result 
    Input:  maxIter: maximum number of iterations
            maxExpansion: maximum distance to expand tree
            env: test environment
    Output: None
    '''

    if env == 1:
        randArea, start, goal, obstacleList = testEnvOne()
    elif env ==2:
        randArea, start, goal, obstacleList = testEnvTwo()

    rrt = RRT(start, goal, obstacleList, randArea, maxIter=maxIter, maxExpansion=maxExpansion)
    path = rrt.planning()
    print('Number of nodes in final tree: ', len(rrt.nodeList))
    print('Number of nodes in final path: ', len(path))
    return None

print("Testing RRT: ")
testRRT(maxIter=2000, maxExpansion=1, env=1)
print("Testing RRT star: ")
testRRTStar(maxIter=1000, maxExpansion=25, searchGamma=40, env=1)