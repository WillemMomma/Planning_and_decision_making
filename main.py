# Importing libraries 
# import env from env.env
from model_predictive_control.MPC import mainMPC
from global_planning.RRT_star import main as mainRRT
# from collision_avoidance.velocity_obstacle import mainCollisionAvoidance
from collision_avoidance.robot_class import Robot
# from env.holonomic_robot_main_test import initEnv, robotMain
import matplotlib.pyplot as plt

from model_predictive_control.uni_cycle_model import UniCycleModel 

# from collision_avoidance.robot_class import Robot

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
    placeholderPos = np.zeros((4,2))
    placeholderPos[1, :] = [4, 4]
    placeholderPos[2, :] = [5, 6]
    placeholderPos[3, :] = [8, 7]

    placeholderVel = np.zeros((4,))
    placeholderOr = np.zeros((4,))
    placeholderTra = np.zeros((100,2))
    currentPositions, currentVelocities, currentOrientations , trajectory = [placeholderPos, placeholderVel,\
                                                                            placeholderOr, placeholderTra] 
        
    '''
    Heb het stuk van momma hier heen verplaatst
    '''
    # env , m , currentPositions, obstacles, currentOrientations, steeringInput = initEnv(goal=True, maps=1)
    
    # trajectory = mainRRT(obstacles,start=currentPositions[0])
    # trajectory = np.array(trajectory).reshape((len(trajectory),2))
    # trajectory = trajectory[::-1]
    '''
    Hier initialiseerd de enviroment
    '''
    # print("TRAJECTORY =", trajectory)

    dummyDataX = np.arange(0 ,50 ,0.1)
    dummyDataY = dummyDataX
    target = np.vstack((dummyDataX,dummyDataY)).T   # Test trajectory
    uni = UniCycleModel(0.1)
    input = np.array([[0,0]])
    jasperPositions = []

    radius = 0.2
    robot_list = []
    
    timestep = 0 
    while timestep < 100:

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
                # trajectory = mainRRT()
                # trajectory = np.array(trajectory).reshape((len(trajectory),2))

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
            Returns the desired velocity and angular velocity 

            INPUT
            timestep -> int : 0
            currentPosition -> list : [x,y]
            currentOrientation -> np.float : 0.0
            trajectory -> np.array() : shape -> (n,2)

            OUTPUT
            currentVelocities[0] -> np.float: 0.0
            angularVelocity -> np.float: 0.0
            """

            currentVelocities[0], angularVelocity = mainMPC(timestep, currentPositions[0,:].tolist(),  currentOrientations[0], target)

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

            # First update the other robots
            for i in range(len(robot_list)):
                if not robot_list[i].our:

                    # Update the position and velocity of the other robots
                    robot_list[i].update_other(currentPositions[i, 0],
                                               currentPositions[i, 1],
                                               currentVelocities[i],
                                               0,
                                               currentOrientations[i])

            # For our robot
            for i in range(len(robot_list)):
                if robot_list[i].our:

                    # Set 0 to a larger value to see some plotting
                    if abs(robot_list[0].x - robot_list[1].x) < 0:
                        robot_list[i].plotting = True
                    else:
                        robot_list[i].plotting = False

                    # Update our robot and check for collision
                    robot_list[i].update_our(currentPositions[i, 0],
                                             currentPositions[i, 1],
                                             currentVelocities[i],
                                             angularVelocity,
                                             currentOrientations[i],
                                             robot_list)

                    # Update the velocity and angular_velocity to be collision free
                    currentVelocities[0] = robot_list[i].output_v
                    angularVelocity = robot_list[i].output_w


            # Godert jij moet deze waarden naar die van jou annpassen
            godert_input = np.array([currentVelocities[0], angularVelocity])
            xytheta = uni.nextX(godert_input.reshape((1,2)))
            xy = xytheta[:2]
            jasperPositions.append(xy.reshape(1,2))
            currentPositions[0,:] = xy.flatten()           
            currentOrientations[0] = xytheta[2]

            # Calculate the desired input for the robot using MPC
            # It is important that all the variables are provided in the correct format @Willem Kolff    
            
            # Willem Kolff
            """
            INPUT : 
            currentPositions : np.array() : shape -> (m, 2)
            currentOrientations : np.array() : shape -> (m,)
            velocity -> np.float : 0.0
            angularVelocity -> np.float : 0.0
            env -> gym.wrappers.order_enforcing.OrderEnforcing
            
            OUTPUT 
            Map -> UNKNOWN
            currentPositions : np.array() : shape -> (m, 2)
            currentVelocities : np.array() : shape -> (m,)
            currentOrientations : np.array() : shape -> (m,)
            """
            # currentPositions, currentVelocities, currentOrientations = robotMain(m, currentPositions, currentVelocities[0], currentOrientations, angularVelocity, steeringInput[timestep], env)
            # Below is the pseudocode provided
            # Please import simulation as well
            # map, currentPositions, currentVelocities, currentOrientations = simulation(velocity, angularVelocity)

            # Check if the final position has been reached
            # print("currentPositions[0,:]", currentPositions[0,:])
            # print("trajectory[-1,:]", trajectory[-1,:])
            # print("Hierooo = ",np.linalg.norm(np.array([currentPositions[0,:]]) - trajectory[-1,:]))
            if np.linalg.norm(np.array([currentPositions[0,:]]) - [1e3, 1e3]) < 1:
               state = 1


            timestep += 1

        # This state signifies that we have finished
        if state == 1:
            print("We have reached our goal")
            run = False

    print("jasperPositions")
    print(np.array(jasperPositions)[0,:],np.array(jasperPositions)[1,:])
    print(np.sum(np.array(jasperPositions), axis = 1 )[:,0])

    plt.scatter(placeholderPos[:, 0], placeholderPos[:, 1], 400)
    plt.plot(np.sum(np.array(jasperPositions), axis = 1 )[:,0],np.sum(np.array(jasperPositions), axis = 1 )[:,1])
    # plt.plot(target[:100,:])
    plt.show()



behaviour()
