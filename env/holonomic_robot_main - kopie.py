import sys
sys.path.append('../')
import gym
import numpy as np

from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle

def initEnv(goal=False):
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
    env.reset(pos=initial_positions,mount_positions=mount_positions)

    # Set goal to follow
    if goal:
        from gym_envs_urdf.scene_objects.goal import dynamicGoal

        env.add_goal(dynamicGoal)
        
    return env
    

def robotMain(pos, vel, currentOrientations, omega, env, render=False, obstacles=False, maps=0, dt = 0.01):

    if obstacles:
        from scene_objects.obstacles import (
            sphereObst1,
            sphereObst2,
            urdfObst1,
            dynamicSphereObst3,
        )

        env.add_obstacle(sphereObst1)
        env.add_obstacle(sphereObst2)
        env.add_obstacle(urdfObst1)
        env.add_obstacle(dynamicSphereObst3)




    if maps == 1:
        from scene_objects.obstacles import (
            walls1,
            boxes1,
            obstacles1
            
        )
        for wall in walls1:
            env.add_shapes(shape_type=wall[0], dim=wall[1], poses_2d=wall[2])

        for box in boxes1:
                env.add_shapes(shape_type=box[0], dim=box[1],mass =box[2], poses_2d=box[3] , place_height =box[4])

        
        for obstacle in obstacles1:
            env.add_obstacle(obstacle)
        
    else:
        pass
        
    v = vel
    omega = omega
    posx = pos[0][0]
    posy = pos[0][1]
    posTheta = currentOrientations[0]
    
    v_vector = np.array([v,0])

    rotation = np.array([[np.cos(posTheta),-np.sin(posTheta)],
                         [np.sin(posTheta),np.cos(posTheta)]])
    v_rotated = rotation @ v_vector
  
    posx_new = posx + v_rotated[0]*dt 
    posy_new = posy + v_rotated[1]*dt 
    posTheta_new = posTheta + omega*dt
    
    pos_new = np.array([[posx_new, posy_new]])
    orientationNew = np.array([posTheta_new])
    action_new = np.append(v_rotated,omega)
    velNew = np.array([vel])


    ob, _, _, _ = env.step(action_new)
    return pos_new, velNew , orientationNew

