# Importing own files
# import env from env.env
from model_predictive_control.MPC import mainMPC
from global_planning.RRT_star import main as mainRRT
from collision_avoidance.robot_class import Robot
# from env.holonomic_robot_main_test import initEnv, robotMain
from model_predictive_control.uni_cycle_model import UniCycleModel 

# Importing libraries
import numpy as np
import matplotlib.pyplot as plt


# # Function for dummy data creation
# def dummydata():
#     # Dummy data
#     start = 0
#     stop = 10
#     dt = 0.1
#     dummyDataX = np.arange(start, stop, dt)
#     dummyDataY = np.ones(len(dummyDataX))
#
#     # Reshaping data
#     X = np.reshape(dummyDataX, (dummyDataX.shape[0], 1))
#     velX = np.zeros(X.shape)
#     Y = np.reshape(dummyDataY, (dummyDataY.shape[0], 1))
#     velY = np.zeros(Y.shape)
#     target = np.concatenate((X, velX, Y, velY), axis= 1)
#     return target


def behaviour():
    """"
    Start the enviroment and run the algorithms

    Input -> None : None
    Ouput -> None : None
    """""

    # Initialization of all robots positions, velocities and orientations
    placeholderPos = np.ones((1,2))
    # placeholderPos[1, :] = [4, 4]
    # placeholderPos[2, :] = [5, 6]
    # placeholderPos[3, :] = [8, 7]

    placeholderVel = np.zeros((1,))
    placeholderOr = np.zeros((1,))
    placeholderTra = np.zeros((100,2))
    currentPositions, currentVelocities, currentOrientations , trajectory = [placeholderPos,
                                                                             placeholderVel,
                                                                             placeholderOr,
                                                                             placeholderTra]


    # Some dummy trajectory should be removed eventually
    dummyDataX = np.arange(0 ,50 ,0.1)
    dummyDataY = dummyDataX
    target = np.vstack((dummyDataX,dummyDataY)).T   # Test trajectory

    # Init for env @Willem Kolff
    # env , m , currentPositions, obstacles, currentOrientations, steeringInput = initEnv(goal=True, maps=1)

    # Init for MPC @Jaspen van Leuven
    uni = UniCycleModel(0.1)
    input = np.array([[0,0]])
    jasperPositions = []

    # Init for collision avoidance @Godert Notten
    radius = 0.2
    robot_list = []

    # Settings for the running loop
    state = 0
    run = True
    timestep = 0

    # Running loop
    # while run == True: # Change to this condition eventually This requires coordination between @Jasper van Leuven & @Willem Momma
    while timestep < len(trajectory):

        # This state signifies the running, and working envirment
        if state == 0: 

            # For the first iteration we have to create a Map of the enviroment and a path to the goal and init the robot_list
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

                # @Willem Kolff & @Willem Momma coordinate on how the obstacles from the map can be passed
                # to the mainRRT algorithm for the path creation!

                # Delete this if your implementation works this is for test purposes
                # trajectory = mainRRT(map) # Should be this line
                trajectory = mainRRT()
                trajectory = np.array(trajectory).reshape((len(trajectory),2))
                print("TRAJECTORY =", trajectory)

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

            # @Jasper van Leuven
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

            currentVelocities[0], angularVelocity = mainMPC(timestep, currentPositions[0,:].tolist(),  currentOrientations[0], trajectory)

            # @Godert Notten
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

                    # Can be changed to show plotting of the velocity obstacles
                    # if abs(robot_list[0].x - robot_list[1].x) < 0:
                    #     robot_list[i].plotting = True
                    # else:
                    #     robot_list[i].plotting = False

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


            # This is now how we update the positions of the robots but this block of code should be replaced
            # in function by the environment @Willem Kolff
            godert_input = np.array([currentVelocities[0], angularVelocity])
            xytheta = uni.nextX(godert_input.reshape((1,2)))
            xy = xytheta[:2]
            jasperPositions.append(xy.reshape(1,2))
            currentPositions[0,:] = xy.flatten()           
            currentOrientations[0] = xytheta[2]

            # @Willem Kolff
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
            # print("currentPositions[0]", currentPositions[0])
            # print("trajectory[-1]", trajectory[-1])
            # print("Hierooo = ",np.linalg.norm(np.array([currentPositions[0]]) - trajectory[-1]))
            if np.linalg.norm(np.array([currentPositions[0,:]]) - trajectory[-1]) < 2:
               state = 1
               print("We have reached our goal")
               run = False


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
