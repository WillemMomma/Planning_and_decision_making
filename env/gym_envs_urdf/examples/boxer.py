import gym
from urdfenvs.robots.boxer import BoxerRobot
import numpy as np


def run_boxer(n_steps=1000, render=False, goal=False, obstacles=False):
    robots = [
        BoxerRobot(mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    action = np.array([0.0, 0.0])
    pos0 = np.array([1.0, 0.2, -1.0])
    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")
    env.add_walls()
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
    
    
    print("Starting episode")
    history = []
    
    for _ in range(n_steps):
        print(_)
        if _ > 100 and _ < 115:
            action += np.array([0,0.2])
        if _ > 200 and _ < 215:
            action += np.array([0,0.2]) * -1
            
        if _ > 300 and _ < 310:
            action += np.array([0.1,0])
        if _ > 400 and _ < 410:
            action += np.array([0.1,0]) * -1            
        
        
        
        if _ > 430 and _ < 450:
            action += np.array([0,0.2])
        if _ > 600 and _ < 620:
            action += np.array([0,0.2]) * -1
            
            
        if _ > 630 and _ < 650:
            action += np.array([0.1,0])
        if _ > 800 and _ < 820:
            action += np.array([0.1,0]) * -1   
        ob, _, _, _ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_boxer(render=True)
