import gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle


    

def run_point_robot(n_steps=1000, render=False, goal=True, obstacles=False, maps=0):
    robots = [
        GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    #action = np.array([6.27078688 , 0.39452447 , 6.28318531])
    action = np.array([0.0,0.0,0.0])   
    pos0 = np.array([0, 0, 270 /180 * np.pi])

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
        
    pos = np.array([0.0,0.0,0.0])
    posx = 0
    posy = 0
    history = []
    posTheta = 0
    angle = 0
    n = 51  # Number of repetitions
    m =50
    o =200
    p = 100
    q = 25
    vector = np.array([2*np.pi, 2*np.pi])
    vector1 = np.array([2*np.pi, -2*np.pi])
    vector2 = np.array([1, 0])
    vector3 = np.array([2*np.pi, -np.pi])
    vector4 = np.array([1, 0])
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
    arr10 = np.insert(arr10, 1, 0, axis=1)

    for _ in range(n_steps):
        

        if _ < (len(arr10)):
            posTheta += (arr10[_][2]/100)

            rotation = np.array([[np.cos(posTheta),-np.sin(posTheta)],
                                 [np.sin(posTheta),np.cos(posTheta)]])
            rotated = rotation.dot(arr10[_][:2])
      
            posx += rotated[0]/100
            posy += rotated[1]/100    
            angle = arr10[_][2]
            action = np.append(rotated,angle)
            if _ <2:
                print(action)

            print("action 2 =====================",action, posx, posy)
        
        
            
        ob, _, _, _ = env.step(action)
        
        history.append(ob)
        
    env.close()
    return history


if __name__ == "__main__":
    run_point_robot(render=True)
