import gym
from urdfenvs.robots.jackal import JackalRobot
import numpy as np


def run_jackal(n_steps=1000, render=False, goal=True, obstacles=True):
    robots = [
        JackalRobot(mode="acc"),
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
    print("Starting episode")
    history = []
    for _ in range(n_steps):
        
        if _ > 20 and _ < 35:
            action += np.array([0.1,0])

        ob, _, _, _ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_jackal(render=True, n_steps=1000)
