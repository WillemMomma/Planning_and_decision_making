import gym
import numpy as np
import itertools
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle
'''
class steering():
    def right(v,r):
        
    def left(v,r):
        
    def straight(v, dist):
'''        

def initEnv(goal=False, obstacles=False, maps=0,):   
    if maps == 1:
        result = []
        robots = [
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
        ns_per_robot = env.ns_per_robot()


        initialPositions = np.array(
            [
                (0,0,0), (0,0,1), (0,0,0), (0,0,np.deg2rad(90))
            ]
        )    
     
        
        mountPositions = np.array(
            [
                #(0,0,0), (-8,6,0), (8,-6,0), (-8,-8.75,0)
                (0,0,0), (-2,0,0), (-3,0,0), (-4,0,0)
            ]
        )
        env.reset(pos=initialPositions,mount_positions=mountPositions)

        #from env.gym_envs_urdf.steering import steeringInput

        from env.gym_envs_urdf.scene_objects.obstacles import (
            walls1,
            #boxes1,
            #obstacles1
            
        )

        for wall in walls1:
            env.add_shapes(shape_type=wall[0], dim=wall[1], poses_2d=wall[2])
            
        for i in range(len(walls1)):
            dimensions = walls1[i][1][0:2]
            coordinates = [coord[:2] for coord in walls1[i][2]]
            
            result.append([[coordinates[0], coordinates[1], dimensions[0], dimensions[1]] for coordinates in coordinates])
            
        obstacles = list(itertools.chain(*result))

            
        '''           
        for box in boxes1:
            env.add_shapes(shape_type=box[0], dim=box[1],mass =box[2], poses_2d=box[3] , place_height =box[4])
        
        for obstacle in obstacles1:
            env.add_obstacle(obstacle)
            
        '''
    else:
        pass

    

    # Set goal to follow
    if goal:
        from env.gym_envs_urdf.scene_objects.goal import dynamicGoal

        env.add_goal(dynamicGoal)
        
    if obstacles:
        from env.gym_envs_urdf.scene_objects.obstacles import (
            sphereObst1,
            sphereObst2,
            urdfObst1,
            dynamicSphereObst3,
        )

        env.add_obstacle(sphereObst1)
        env.add_obstacle(sphereObst2)
        env.add_obstacle(urdfObst1)
        env.add_obstacle(dynamicSphereObst3)
        
    return env , m, mountPositions[:,:2], obstacles, initialPositions[:,2]+np.deg2rad(90)#, #, steeringInput
    

def robotMain(pos, vel, current_orientations, omega, otherRobots, env, render=False, dt=0.01):
    # Extract position and orientation from the inputs
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
    pos_new = np.array([[x_new, y_new]])
    orientation_new = np.array([theta_new])
    
    # Create the action array for the current robot
    action_new = np.append(v_rotated, omega)
    
    # Create an action array for the other robots
    action_other_bots = np.zeros(9)
    
    # Concatenate the action arrays for all robots
    action_new = np.concatenate((action_new, action_other_bots))
    vel_new = np.array([vel])
    
    # Step the environment and return the new position, velocity, and orientation
    ob, _, _, _ = env.step(action_new)
    return pos_new, vel_new, orientation_new

