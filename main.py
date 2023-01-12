# Importing libraries
from model_predictive_control.MPC import mainMPC
from global_planning.RRT_star import main as mainRRT
from collision_avoidance.robot_class import Robot
from env.holonomic_robot_main import initMap, initEnv, robotMain

# Importing libraries
import numpy as np
import random
import matplotlib.pyplot as plt


def behaviour():
    """
    Start the enviroment and run the algorithms

    Input -> None : None
    Ouput -> None : None
    """
    # Choose your map
    # map = 0 -> test map
    # map = 1 -> warehouse multiple robots
    map = 1

    # Init for collision avoidance @Godert Notten
    radius = 0.2
    robot_list = []

    # Settings for the running loop
    state = 0
    timestep = 0
    start_position = [random.randrange(-10, 10+1, 2), random.randrange(-10, 10+1, 2)]
    goal_position = [random.randrange(-10, 10+1, 2), random.randrange(-10, 10+1, 2)]
    other_robots = True

    # Running loop
    while timestep < 5000:  # This is to freeze the final situation for reference of the user
        if state == 0:

            # For the first iteration we have to create a Map of the environment and a path to the goal
            if timestep == 0:
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

                # Initialize the map
                # Other robots, map_number, start_position
                mountPositions, obstacles = initMap(other_robots, map, start_position)
                # Create the trajectory 
                trajectory = mainRRT(obstacles, mountPositions[0, 0:2], goal_position)
                trajectory = np.array(trajectory).reshape((len(trajectory), 2))
                # Create the Enviroment

                env, m, currentPositions, currentOrientations, steeringInput = initEnv(mountPositions,
                                                                                       trajectory,
                                                                                       goal=True,
                                                                                       otherrobots=other_robots,
                                                                                       maps=map,
                                                                                       dt=0.01)
                currentVelocities = np.zeros((m,))
                currentAngularVelocities = np.zeros((m,))

                for i in range(m):
                    if i == 0:
                        robot_list.append(Robot(currentPositions[i, 0],
                                                currentPositions[i, 1],
                                                radius,
                                                currentVelocities[i],
                                                currentAngularVelocities[i],
                                                currentOrientations[i],
                                                True))
                    else:
                        robot_list.append(Robot(currentPositions[i, 0],
                                                currentPositions[i, 1],
                                                radius,
                                                currentVelocities[i],
                                                currentAngularVelocities[i],
                                                currentOrientations[i],
                                                False))

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

            currentVelocities[0], currentAngularVelocities[0] = mainMPC(timestep, currentPositions[0,:].tolist(),  currentOrientations[0], trajectory)

            """
            INPUT

            m = number of robots in env including our own robot !!!

            robot_list : list of length m filled with Robot objects
            # positions -> np.array() : shape -> (m, 2)
            # velocities -> np.array() : shape -> (m,)
            # angularVelocities -> np.float: 0.0
            # orientations -> np.array() : shape -> (m,)

            #cOUTPUT
            #robot_list : list of length m filled with Robot objects
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
                                                currentAngularVelocities[i],
                                                currentOrientations[i])

            # For our robot
            for i in range(len(robot_list)):
               if robot_list[i].our:

                    # Update our robot and check for collision
                    robot_list[i].update_our(currentPositions[i, 0],
                                              currentPositions[i, 1],
                                              currentVelocities[i],
                                              currentAngularVelocities[0],
                                              currentOrientations[i],
                                              robot_list)

                    # Update the velocity and angular_velocity to be collision free
                    currentVelocities[0] = robot_list[i].output_v
                    currentAngularVelocities[0] = robot_list[i].output_w

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
            currentPositions, angularVelocities, currentVelocities, currentOrientations = robotMain(mountPositions, m, currentPositions, currentVelocities[0], currentOrientations, currentAngularVelocities[0], steeringInput[timestep], env)

            # Check if the final position has been reached
            if np.linalg.norm(np.array([currentPositions[0,:]]) - trajectory[-1]) < 0.5:
               state = 1
               print("We have reached our goal")

            timestep += 1

        # This state signifies that we have finished
        if state == 1:
            continue


behaviour()
