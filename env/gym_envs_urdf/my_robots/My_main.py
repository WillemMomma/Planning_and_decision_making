# -*- coding: utf-8 -*-
"""
Created on Thu Dec 12 19:26:06 2022

@author: wille
"""
import gym
import numpy as np
from urdfenvs.robots.generic_urdf import GenericUrdfReacher

from point_robot_main import robotMain as robotMain



####################################### action input ###############################
n = 51  # Number of repetitions
m =50
o =200
p = 100
q = 25
vector = np.array([2*np.pi, 2*np.pi])
vector1 = np.array([2*np.pi, -2*np.pi])
vector2 = np.array([1, 0])
vector3 = np.array([2*np.pi, -np.pi])
vector4 = np.array([1, 0])
vector5 = np.array([2*np.pi, -2*np.pi])
vector6 = np.array([0, 0])


arr = np.tile(vector, (n, 1))
arr1 = np.tile(vector1, (m, 1))
arr2 = np.tile(vector2, (o, 1))
arr3 = np.tile(vector3, (p, 1))
arr4 = np.tile(vector4, (p, 1))
arr5 = np.tile(vector5, (q, 1))
arr6 = np.tile(vector6, (q, 1))

arr10 = np.concatenate((arr,arr1,arr2,arr3,arr4,arr5,arr6),axis=0)

######################################## Main Loop #################################

def initEnv(goal=False):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=True
    )
    
    
    ####################################### Initializing ###############################
    action = np.array([0.0,0.0,0.0])   
    pos0 = np.array([0, 0, 270 /180 * np.pi])
    vel0 = np.array([0.0, 0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)    
    
    # Initialize the robot's position
    pos = (0, 0, 0)
    
    # Set the action to be taken by the robot
    action = (1, 0)
    
    # Set the maximum number of iterations for the while loop
    max_iter = 100
    
    # Set the counter variable to zero
    i = 0
    
    
    # Set goal to follow
    if goal:
        from gym_envs_urdf.scene_objects.goal import dynamicGoal

        env.add_goal(dynamicGoal)
    
    # Run the while loop
    while i < max_iter:
        # Call the run_robot function, passing in the current action
        action, pos = robotMain(pos, arr10[i], env)
        i += 1
if __name__ == "__main__":
    initEnv(goal=True)

