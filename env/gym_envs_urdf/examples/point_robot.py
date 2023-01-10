import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np

def run_point_robot(n_steps=1000, render=False, goal=True, obstacles=False):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    action = np.array([0, 0.0, 0.0])
    pos0 = np.array([0.0, 0.0, 0.0])
    vel0 = np.array([0.0, 3.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)
    print(f"Initial observation : {ob}")



    poseStorage  = [
                  [-0.75, 3.5, 0],
                  [-2.75, 3.5, 0],
                  [-4.75, 3.5, 0],
                  [-6.75, 3.5, 0],
            ]
    
    poseWall1  = [
              [-7.75, 1.25, 0]

        ]
    
    poseWall2  = [
              [-4, 7.1, np.pi * 0.5]

        ]
    poseWall2oxes  = [
              [-3, -1.5, 0],
              [-6, -1.5, 0],
              [-3, -3, 0],
              [-6, -3, 0],

        ]
    env.add_shapes(shape_type="GEOM_BOX", dim=[0.4, 6, 0.25], poses_2d=poseStorage)
    env.add_shapes(shape_type="GEOM_BOX", dim=[0.2, 11.5, 1], poses_2d=poseWall1)
    env.add_shapes(shape_type="GEOM_BOX", dim=[0.2, 8.35, 1], poses_2d=poseWall2)
    env.add_shapes(shape_type="GEOM_BOX", dim=[0.2, 0.2, 0.2],mass = 0.3, poses_2d=poseWall2oxes , place_height = 0.3)
    env.add_shapes(shape_type="GEOM_BOX", dim=[0.2, 0.2, 0.2],mass = 0.3, poses_2d=poseWall2oxes , place_height = 0.6)
    env.add_shapes(shape_type="GEOM_BOX", dim=[0.2, 0.2, 0.2],mass = 0.3, poses_2d=poseWall2oxes , place_height = 0.9)


    if obstacles:
        from examples.scene_objects.obstacles import (
            sphereObst1,
            sphereObst2,
            urdfObst1,
            dynamicSphereObst3,
        )

        env.add_obstacle(sphereObst1)
        env.add_obstacle(sphereObst2)
        env.add_obstacle(urdfObst1)
        env.add_obstacle(dynamicSphereObst3)
    if goal:
        from examples.scene_objects.goal import dynamicGoal

        print(dynamicGoal)
        hoi = env.add_goal(dynamicGoal)
        print("hoi ===========", hoi)
    history = []
    for _ in range(n_steps):
        

        if _ > 50 and _ < 88:
            action += np.array([0.1, 0.0, 0]) * -1
            
        if _ > 100 and _ < 138:
            action += np.array([0.1, 0.0, 0]) 
            
            
        if _ > 150 and _ < 225:
            action += np.array([0.0, 0.1, 0]) 
            
        if _ > 240 and _ < 315:
            action += np.array([0.0, 0.1, 0]) * -1
            
            
        if _ > 350 and _ < 380:
            action += np.array([0.1, 0.0, 0]) * -1
            
        if _ > 400 and _ < 430:
            action += np.array([0.1, 0.0, 0]) 
            
            
            
        if _ > 430 and _ < 500:
            action += np.array([0.0, 0.1, 0]) * -1
            
        if _ > 600 and _ < 610:
            action += np.array([0.3, 0.0, 0]) 
            
        if _ > 690 and _ < 700:
            action += np.array([-0.1, 0.0, 0]) 
        if _ > 700 and _ < 770:
            action += np.array([0.0, 0.1, 0]) 
            
        ob, _, _, _ = env.step(action)
        
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_point_robot(render=True)
