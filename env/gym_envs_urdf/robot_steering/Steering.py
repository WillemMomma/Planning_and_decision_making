# -*- coding: utf-8 -*-
"""
Created on Sun Jan  1 18:20:57 2023

@author: wille
"""

import numpy as np
class steering:
    def straight(v,n):
        vector = np.array([v,0])      
        m = int(n/v *100)
        arr = np.tile(vector, (m, 1))
        return(arr)
        
    def rightCorner(v,r,deg):
        omega = v/r
        m = int(np.pi*100*deg)
        vector = np.array([v,omega])
        arr = np.tile(vector, (m, 1))
        return(arr)
    def stop(t):
        vector = np.array([0, 0])
        m = 100 * t
        arr = np.tile(vector, (m, 1))
        return(arr)        
        
    
arr = np.concatenate(steering.straight(1, 3), steering.rightCorner(1,3,90), steering.straight(1, 3))
        





        n = 51  # Number of repetitions
        m =50
        o =200
        p = 100
        q = 25
        vector = np.array([2*np.pi, 2*np.pi])
        vector1 = np.array([2*np.pi, -2*np.pi])
        vector2 = np.array([4, 0])
        vector3 = np.array([2*np.pi, -np.pi])
        vector4 = np.array([5, 0])
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
        
        placeholderPos = np.zeros((10,2))
        placeholderVel = np.zeros((10,))
        placeholderOr = np.zeros((10,))
        placeholderTra = np.zeros((100,2))
        currentPositions, currentVelocities, currentOrientations , trajectory = [placeholderPos, placeholderVel,\
                                                                                placeholderOr, placeholderTra] 
        angularVelocity = np.float64(0) 
    
          