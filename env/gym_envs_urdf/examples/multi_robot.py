import gym
import numpy as np
from urdfenvs.robots.generic_urdf import GenericUrdfReacher


def run_multi_robot(n_steps=1, render=False, obstacles=False, goal=False, maps=1):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel")
    ]


    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    n = env.n()
    action = np.ones(n) * -0.2
    pos0 = np.zeros(n)
    pos0[1] = -0.0
    ns_per_robot = env.ns_per_robot()
    n_per_robot = env.n_per_robot()

    initial_positions = np.array([np.zeros(n) for n in ns_per_robot])
    for i in range(len(initial_positions)):
        if ns_per_robot[i] != n_per_robot[i]:
            initial_positions[i][0:2] = np.array([0.0, i])
    
    print("HHHHHHHHHHHHHHIIIIIIIIEEEEEEEERRRRRRR",initial_positions)
    mount_positions = np.array(
        [
            np.array([0.0, i, 0.0]) for i in range(len(ns_per_robot))
        ]
    )

    ob = env.reset(pos=initial_positions,mount_positions=mount_positions)
    if goal:
        from scene_objects.goal import dynamicGoal
        env.add_goal(dynamicGoal)

    if obstacles:
        from scene_objects.obstacles import dynamicSphereObst2
        env.add_obstacle(dynamicSphereObst2)
        
        
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
     

    print("Starting episode")
    history = []
    for _ in range(n_steps):
        if _ > 50 and _ < 55:
            action += np.array([0.1, 0.0, 0, 1 , .8, 1, .9, 1, 1,1,1,1]) 
        if _ == 90:
            action = np.array([0.1, 0.0, 0, 0,0,0,0,0,0,0,0,0]) 

        ob, _, _, _ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_multi_robot(render=True, obstacles=True, goal=True)
