import gym
import numpy as np
import itertools
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle

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
    

def initEnv(goal=False, obstacles=False, maps=0):   

    if maps == 1:
        result = []
        robots = [
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel")
        ]
        
        m = len(robots)
        env = gym.make(
            "urdf-env-v0",
            dt=0.01, robots=robots, render=True
        )
        
        n = env.n()       
        pos0 = np.zeros(n)
        pos0[1] = -0.0

        initialPositions = np.array(
            [
                (0,0,np.deg2rad(180)), (0,0,0), (0,0,np.pi), (0,0,np.deg2rad(90)),(0,0,np.deg2rad(270)),(0,0,np.deg2rad(270)),(0,0,np.deg2rad(90))
            ]
        )    
     
        
        mountPositions = np.array(
            [
                (4,10,0), (6,-8,0), (-6,8,0), (4,0.5,0),(-6,-0.5,0),(-12,-0.5,0),(12,0.5,0)
                
            ]
        )
        env.reset(pos=initialPositions,mount_positions=mountPositions)

        arr1 = np.concatenate([steering.straight(6, 6.5), steering.left(6,1,92), steering.straight(6, 9), steering.right(6,2,91),steering.straight(6, 0.1),steering.left(6, 2, 25),steering.right(6, 2, 25),steering.straight(6, 2),steering.right(6, 2, 25),steering.left(6, 2, 25),steering.left(6, 1, 180),steering.straight(6, 14),steering.left(6, 1, 91),steering.straight(6, 10),steering.left(6, 1, 91),steering.straight(6, 6), steering.right(6,2,91), steering.straight(6, 3)])
        arr2 = np.concatenate([steering.straight(6, 6.5), steering.left(6,1,92), steering.straight(6, 5), steering.right(6,2,91),steering.straight(6, 5.5),steering.left(6, 1, 180),steering.straight(6, 14),steering.left(6, 1, 91),steering.straight(6, 10),steering.left(6, 1, 91),steering.straight(6, 6), steering.right(6,2,91), steering.straight(6, 3)])
        arr3 = np.concatenate([steering.straight(6, 7),steering.right(6,1,91),steering.straight(6, 5.5),steering.left(6, 1, 180),steering.straight(6, 2),steering.left(6, 2, 25),steering.right(6, 2, 25),steering.straight(6, 2),steering.right(6, 2, 25),steering.left(6, 2, 25),steering.straight(4, 7),steering.left(6, 1, 91),steering.straight(6, 10),steering.left(6, 1, 91),steering.straight(6, 6), steering.right(6,2,91), steering.straight(6, 3)])
        arr4 = np.concatenate([steering.straight(6, 3),steering.right(6,1,91),steering.straight(6, 5.5),steering.left(6, 1, 180),steering.straight(6, 2),steering.left(6, 2, 25),steering.right(6, 2, 25),steering.straight(6, 2),steering.right(6, 2, 25),steering.left(6, 2, 25),steering.straight(4, 6),steering.left(6, 1, 91),steering.straight(6, 7),steering.left(6, 1, 91),steering.straight(6, 6), steering.right(6,2,91), steering.straight(6, 3)])
        arr5 = np.concatenate([steering.stop(4), steering.straight(4, 26)])
        arr6 = np.concatenate([steering.stop(5), steering.straight(4, 26)])

        max_len = max(arr1.shape[0], arr2.shape[0], arr3.shape[0],arr4.shape[0],arr5.shape[0],arr6.shape[0])
        
        arr1 = np.pad(arr1, [(0, max_len - arr1.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr2 = np.pad(arr2, [(0, max_len - arr2.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr3 = np.pad(arr3, [(0, max_len - arr3.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr4 = np.pad(arr4, [(0, max_len - arr4.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr5 = np.pad(arr5, [(0, max_len - arr5.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr6 = np.pad(arr6, [(0, max_len - arr6.shape[0]), (0, 0)], 'constant', constant_values=0)
        
        steeringInput = np.concatenate([arr1,arr2,arr3,arr4,arr5,arr6],axis=1)
        
        from env.gym_envs_urdf.scene_objects.obstacles import walls1

    
        for wall in walls1:
            env.add_shapes(shape_type=wall[0], dim=wall[1], poses_2d=wall[2])
            
        for i in range(len(walls1)):
            dimensions = walls1[i][1][0:2]
            coordinates = [coord[:2] for coord in walls1[i][2]]
            
            result.append([[coordinates[0] - 0.5 * dimensions[0] , coordinates[1] - 0.5 * dimensions[1], 
                            coordinates[0] + 0.5 * dimensions[0] , coordinates[1] + 0.5 * dimensions[1]] for coordinates in coordinates])
            
        obstacles = list(itertools.chain(*result))

    else:
        pass


    return env , m, mountPositions[:,:2], initialPositions[:,2]+np.deg2rad(90), obstacles, steeringInput
    



def robotMain(m, pos, vel, current_orientations, omega, otherRobots, env, dt=0.1):
    x = pos[0][0]
    y = pos[0][1]
    theta = current_orientations[0]
    # Calculate the velocity vector in the original orientation
    v_vector = np.array([vel, 0])
    
    # Create a rotation matrix to rotate the velocity vector
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])
    # Rotate the velocity vector
    v_rotated = rotation_matrix @ v_vector
    
    # Calculate the new position and orientation
    x_new = x + v_rotated[0] * dt
    y_new = y + v_rotated[1] * dt
    theta_new = theta + omega * dt
    
    # Create the new position and orientation arrays
    vel_new = np.array([vel])
    pos_new = np.array([[x_new, y_new]])
    orientation_new = np.array([theta_new])
    
    # Create the action array for the current robot
    action_new = np.append(v_rotated, omega)
    
    
    result = otherRobots.reshape((m-1, 3))

    # Extract position and orientation from the inputs
    for i in range(m-1):
        x = pos[i+1][0]
        y = pos[i+1][1]
        theta = current_orientations[i+1]
        v_vector = result[i][0:2]
        omega = result[i][2]
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])
        # Rotate the velocity vector
        v_rotated = rotation_matrix @ v_vector
        
        # Calculate the new position and orientation
        x_new = x + v_rotated[0] * dt
        y_new = y + v_rotated[1] * dt
        theta_new = theta + omega * dt
        
        vel_new = np.append(vel_new, result[i][0])
        pos_new = np.append(pos_new, x_new)
        pos_new = np.append(pos_new, y_new)
        orientation_new = np.append(orientation_new, theta_new)
        
        action_new = np.append(action_new, v_rotated)
        action_new = np.append(action_new, omega)
    
    pos_new = pos_new.reshape((m,2))        
    
    # Step the environment and return the new position, velocity, and orientation
    ob, _, _, _ = env.step(action_new)
    return pos_new, vel_new, orientation_new

