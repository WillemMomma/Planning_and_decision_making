# -*- coding: utf-8 -*-
"""
Created on Fri Dec 30 17:17:42 2022

@author: wille
"""
import numpy as np
from env.holonomic_robot_main import initEnv, robotMain
#from point_robot_main import robotMain as robotMain

#currentPositions, currentVelocities, currentOrientations , trajectory = [placeholderPos, placeholderVel,\
                                                                       # placeholderOr, placeholderTra] 
angularVelocity = np.float64(0) 

import numpy as np
class steering:
    def straight(v,n):
        vector = np.array([v,0,0])      
        m = int(n/v *100)
        arr = np.tile(vector, (m, 1))
        return(arr)
        
    def left(v,r,deg):
        omega = v/r
        m = int((np.pi*100*deg)/(180*omega))
        vector = np.array([v,0,omega])
        arr = np.tile(vector, (m, 1))
        return(arr)
    def right(v,r,deg):
        omega = v/r
        m = int((np.pi*100*deg)/(180*omega))
        vector = np.array([v,0,-omega])
        arr = np.tile(vector, (m, 1))
        return(arr)   
    def stop(t):
        vector = np.array([0, 0, 0])
        m = 100 * t
        arr = np.tile(vector, (m, 1))
        return(arr)        
        
arr5 = np.concatenate([steering.stop(20)])




# Set the maximum number of iterations for the while loop
max_iter = 2000
# Set the counter variable to zero
i = 0

env , m , currentPositions, currentOrientations, obstacles, steeringInput = initEnv(goal=False, maps=1)

while i < max_iter:
    # Call the run_robot function, passing in the current action
    currentPositions, currentVelocities, currentOrientations = robotMain(m, currentPositions, arr5[i][0], currentOrientations, arr5[i][2], steeringInput[i], env)
    if i% 10 == 0:
        print(currentPositions[1])
    i += 1