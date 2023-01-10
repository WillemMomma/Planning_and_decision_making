# -*- coding: utf-8 -*-
"""
Created on Thu Dec 12 19:26:06 2022

@author: wille
"""
import gym
import numpy as np
from urdfenvs.robots.generic_urdf import GenericUrdfReacher

from multi_point_robot_main import run_point_robots as run_point_robots
robots = [
    GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel")
]


env = gym.make(
    "urdf-env-v0",
    dt=0.01, robots=robots, render=True
)


n = env.n()
print(n)
action = np.ones(n)
pos0 = np.zeros(n)
pos0[1] = -0.0
ns_per_robot = env.ns_per_robot()
n_per_robot = env.n_per_robot()
initial_positions = np.array([np.zeros(n) for n in ns_per_robot])
for i in range(len(initial_positions)):
    if ns_per_robot[i] != n_per_robot[i]:
        initial_positions[i][0:2] = np.array([0.0, i])
mount_positions = np.array(
    [
        np.array([0.0, i, 0.0]) for i in range(len(ns_per_robot))
    ]
)
ob = env.reset(pos=initial_positions,mount_positions=mount_positions)


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

def main(goal=False):
    # Initialize the robot's position
    pos = initial_positions
    
    # Set the action to be taken by the robot
    action = np.ones(n)
    
    # Set the maximum number of iterations for the while loop
    max_iter = 1000
    
    # Set the counter variable to zero
    i = 0
    
    # Run the while loop
    while i < max_iter:
        # Call the run_robot function, passing in the current action
        action, pos = run_point_robots(pos, arr10[i], env)
        i += 1
if __name__ == "__main__":
    main(goal=True)

