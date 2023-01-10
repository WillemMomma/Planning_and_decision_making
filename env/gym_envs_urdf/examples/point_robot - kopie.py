import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle

'''
def rotationMatrix(vector,_):
    if _ == 0:
    posTheta = vector[2]*_
    rotation = np.array([[np.cos(posTheta),-np.sin(posTheta)],
                         [np.sin(posTheta),np.cos(posTheta)]])
    rotated = rotation.dot(vector[:2])
    
    output += np.append(rotated,posTheta)
    
    return output
    
'''

def run_point_robot(n_steps=1000, render=False, goal=True, obstacles=False, maps=0):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    action = np.array([0, 0.0, 1])
    pos0 = np.array([0.0, 0.0, 0.0])

    vel0 = np.array([0.0, 0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)
    print(f"Initial observation : {ob}")

    

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

    if goal:
        from scene_objects.goal import dynamicGoal

        env.add_goal(dynamicGoal)


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
        print('start')
        #env.add_walls()
        

    history = []
    
    for _ in range(n_steps):
        action = np.array([0, 0.1, 0.05])

        posTheta = action[2]*_
        angle = posTheta*180 / np.pi
        print(_/100, angle/100)
        
        '''      
        if _ % 100 == 0 and _ < 400:
            n = _/100
            dynamicObst2Dict = {
    "type": "analyticSphere",
    "geometry": {"trajectory": movements[int(n)], "radius": 0.2},
            }
            dynamicSphereObst2 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst2Dict)
            env.add_obstacle(dynamicSphereObst2)
            
            
  
        
        if _ > 50 and _ < 88:
            action += np.array([0.3, 0.0, 0]) * -1

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
       '''           
        ob, _, _, _ = env.step(action)
        
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_point_robot(render=True)
