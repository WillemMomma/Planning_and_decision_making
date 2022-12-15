# Importing libraries 
# import env from env.env
from model_predictive_control.MPC import Run
from global_planning.RRT_star import main_rrt
from collision_avoidance.velocity_obstacle import main_collision
# from collision_avoidance.robot_class import Robot

import numpy as np

def behaviour():
    """
    Start the enviroment and run the algorithms

    input -> None : None
    Ouput -> None : None
    """
    # Initialize enviroment here
    # map = 0    #calling the map function inside the env
    # map = env(map)
    # states = env(states)

    # Genrate the global trajectory here
    # trajectory = main_rrt()
    # print(trajectory)

    # Start in the correct state
    state = 0
    run = True

    # Dummy data
    start = 0
    stop = 10
    dt = 0.1
    dummyDataX = np.arange(start, stop, dt)
    dummyDataY = np.ones(len(dummyDataX))

    # Reshaping data
    X = np.reshape(dummyDataX, (dummyDataX.shape[0], 1))
    velX = np.zeros(X.shape)
    Y = np.reshape(dummyDataY, (dummyDataY.shape[0], 1))
    velY = np.zeros(Y.shape)
    target = np.concatenate((X, velX, Y, velY), axis= 1)


    while run:
        timestep = 0

        if state == 0:  # running

            collision_avoidance_velocity = main_collision(*target[timestep], 0)

            if timestep == 0:
                currentState = np.array([[0], [0], [0], [0]])
            else:
                currentState = target[timestep-1 , :]

            print(timestep)
            print(currentState.shape , target.shape)
            action = Run(timestep, currentState , target)

            print(action)

            # collisionFreePath = collisionAvoidance(trajectory, states, timestep)
            # action = MPC(collisionFreePath["trajectory"], collisionFreePath["velocites"])
            # states = env(action)
#
#             # Check if the state has chaged
#             state  = collisionFreePath["status"]
#
#             """"
#             collisionFreePath = {trajectory : np.array([]),
#                                  velocities : np.array([])
#                                  status     : Int,
#                                  done       : Int}
#             """
#
#         if state == 1: # Calculate new trajectory
#             trajectory = globalPlanning(states)
#             state = trajectory["status"]
#
#         if state == 2: # No possible solution
#             print("No path can be found")
#             run = False
#
#         if state == 3: # No possible solution
#             print("Finished")
#             run = False
#
        timestep += 1
#
#     # Closing the enviroment
#     # env(close)
#     return
# #
#
# if __name__ == "__main__":
#     behaviour()

behaviour()
