
# Description: RRT star path planning algorithm
import random 
import math
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT_star:
    ''' 
    Class for RRT star planning 
    Goal: find a path from start to goal avoiding obstacles
    Returns: path as a list of nodes [[x1, y1], [x2, y2], ...]
    Also plots the growing tree, final path and obstacles
    '''
    def __init__(self, start, goal, obstacle_list, rand_area, max_iter=500):
        '''
        Initialize the class variables
        Input: start: start node
                goal: goal node
                obstacle_list: list of obstacles as vertices of polygons [[x1, y1], [x2, y2], ...]
                rand_area: random sampling area equal to map size
                max_iter: maximum number of expansions
        Output: None
        '''

        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list

    def planning(self, animation=True):
        '''
        Path planning
        Input: animation: flag for animation on or off
        Output: path as a list of nodes [[x1, y1], [x2, y2], ...]
        '''
        self.node_list = [self.start]
        for i in range(self.max_iter):
            q_rand = self.get_random_point()
            nearest_node = self.get_nearest_node(self.node_list, q_rand) # index of nearest node
            theta = math.atan2(q_rand[1] - nearest_node.y, q_rand[0] - nearest_node.x) # angle between nearest node and random point
            new_node = self.steer(nearest_node, theta)
            if self.check_collision(new_node, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
            if animation:
                self.draw_graph(rnd)
        path = self.generate_final_course(len(self.node_list) - 1)
        return path

        