# Importing libraries 
# import env from env.env
from model_predictive_control.MPC import mainMPC
from global_planning.RRT_star import main as mainRRT
from collision_avoidance.velocity_obstacle import mainCollisionAvoidance
from collision_avoidance.robot_class import Robot

import numpy as np

def dummydata():
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
    return target


def behaviour():
    """
    Start the enviroment and run the algorithms

    Input -> None : None
    Ouput -> None : None
    """

    # Start in the correct state
    state = 0
    run = True
    placeholderPos = np.zeros((10,2))
    placeholderVel = np.zeros((10,))
    placeholderOr = np.zeros((10,))
    placeholderTra = np.zeros((100,2))
    currentPositions , currentVelocities,currentOrientations , trajectory = [placeholderPos, placeholderVel,\
                                                                            placeholderOr, placeholderTra]
    # Init robot list
    robot_list = []
    radius = 0.2

    # Init timestep with value 0
    timestep = 0

    # Run loop
    while run:

        # This state signifies the running, and working envirment
        if state == 0: 

            # For the first iteration we have to create a Map of the enviroment and a path to the goal 
            if timestep == 0:   
                velocity = np.float64(0)
                angularVelocity = np.float64(0)

                # Willem Kolff
                """
                INPUT : 
                velocity -> np.float : 0.0
                angularVelocity -> np.float : 0.0
                
                OUTPUT 
                Map -> UNKNOWN
                currentPositions : np.array() : shape -> (m, 2)
                currentVelocities : np.array() : shape -> (m,)
                currentOrientations : np.array() : shape -> (m,)
                """

                # Below is the pseudocode provided
                # Please import simulation as well
                # map, currentPositions, currentVelocities, currentOrientations = simulation(velocity, angularVelocity)

                # Willem kOLFF
                # Please alter the code in the mainRRT as well to accept the map input
                # This funtion should recieve the map as input @Willem Kolff

                # Below is the pseudocode provided
                # trajectory = mainRRT(map)

                # Delete this if your implementation works this is for test purposes
                trajectory = mainRRT()
                trajectory = np.array(trajectory).reshape((len(trajectory),2))

                for i in range(len(currentPositions)):
                    if i == 0:
                        robot_list.append(Robot(currentPositions[i, 0],
                                                currentPositions[i, 1],
                                                radius,
                                                currentVelocities[i],
                                                angularVelocity,
                                                currentOrientations[i],
                                                True))
                    else:
                        robot_list.append(Robot(currentPositions[i, 0],
                                                currentPositions[i, 1],
                                                radius,
                                                currentVelocities[i],
                                                0,
                                                currentOrientations[i],
                                                False))


            
            # Jasper
            """
            INPUT
            timestep -> int : 0
            currentPosition -> list : [x,y]
            currentOrientation -> np.float : 0.0
            trajectory -> np.array() : shape -> (n,2)

            OUTPUT
            currentVelocities[0] -> np.float: 0.0
            angularVelocity -> np.float: 0.0
            """

            currentVelocities[0], angularVelocity = mainMPC(timestep, currentPositions[0,:].tolist(),  currentOrientations[0], trajectory ) 

            # Godert
            """
            INPUT

            m = number of robots in env including our own robot !!!

            robot_list : list of length m filled with Robot objects
            # positions -> np.array() : shape -> (m, 2)
            # velocities -> np.array() : shape -> (m,)
            # angularVelocities -> np.float: 0.0
            # orientations -> np.array() : shape -> (m,)

            OUTPUT
            robot_list : list of length m filled with Robot objects
            # velocities -> np.float : 0
            # angularVelocities -> np.float() : 0 
            """

            for i in range(len(robot_list)):
                if i == 0:
                    robot_list[i].update(currentPositions[i, 0],
                                         currentPositions[i, 1],
                                         currentVelocities[i],
                                         angularVelocity,
                                         currentOrientations[i])
                else:
                    robot_list[i].update(currentPositions[i, 0],
                                         currentPositions[i, 1],
                                         currentVelocities[i],
                                         0,
                                         currentOrientations[i])

            robot_list = mainCollisionAvoidance(robot_list)
            velocity = robot_list[0].output_v
            angularVelocity = robot_list[0].output_w

            # Calculate the desired input for the robot using MPC
            # It is important that all the variables are provided in the correct format @Willem Kolff    
            
            # Willem Kolff
            """
            INPUT : 
            velocity -> np.float : 0.0
            angularVelocity -> np.float : 0.0
            
            OUTPUT 
            Map -> UNKNOWN
            currentPositions : np.array() : shape -> (m, 2)
            currentVelocities : np.array() : shape -> (m,)
            currentOrientations : np.array() : shape -> (m,)
            """

            # Below is the pseudocode provided
            # Please import simulation as well
            # map, currentPositions, currentVelocities, currentOrientations = simulation(velocity, angularVelocity)

            # Check if the final position has been reached
            if np.linalg.norm(np.array([currentPositions[0,:]]) - trajectory[:,-1]):
                state = 1

        # This state signifies that we have finished
        if state == 1:
            print("We have reached our goal")
            run = False

        timestep += 1


behaviour()
