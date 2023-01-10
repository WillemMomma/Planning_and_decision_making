import numpy as np
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle


    

def robotMain(pos, action, env, n_steps=1000, render=False, obstacles=False, maps=0, dt = 0.01):

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
        
    v = action[0] 
    omega = action[1]
    posx = pos[0]
    posy = pos[1]
    posTheta = pos[2]
    
    v_vector = np.array([v,0])

    rotation = np.array([[np.cos(posTheta),-np.sin(posTheta)],
                         [np.sin(posTheta),np.cos(posTheta)]])
    v_rotated = rotation @ v_vector
  
    posx_new = posx + v_rotated[0]*dt 
    posy_new = posy + v_rotated[1]*dt 
    posTheta_new = posTheta + omega*dt
    
    pos_new = np.array([posx_new, posy_new, posTheta_new])
    action_new = np.append(v_rotated,omega)
    
    ob, _, _, _ = env.step(action_new)
    return action_new, pos_new       


