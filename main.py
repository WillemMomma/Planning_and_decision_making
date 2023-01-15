# Importing libraries
import time

from model_predictive_control.MPC import mainMPC
from global_planning.RRT_star import main as mainRRT
from global_planning.obstacles import ObstacleRectangle
from collision_avoidance.robot_class import Robot
from env.holonomic_robot_main import initMap, initEnv, robotMain

# Importing libraries
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Patch


def behaviour():
    """
    Start the enviroment and run the algorithms

    Input -> None : None
    Ouput -> None : None
    """
    # Choose your map
    # map = 0 -> test map
    # map = 1 -> warehouse multiple robots
    # map = 2 -> warehouse with more space between racks
    # map = 3 -> only robots
    map = 3
    margin = 0

    # Setup per map
    if map == 1:
        start_position = [random.randrange(-10, 10 + 1, 2), random.randrange(-10, 10 + 1, 2)]
        goal_position = [random.randrange(-10, 10 + 1, 2), random.randrange(-10, 10 + 1, 2)]
        margin = 0.4
    elif map == 2:
        x_positions = [-7, -3, 1, 5, 9]
        start_position = [random.choice(x_positions), random.randrange(-10, 10 + 1, 2)]
        goal_position = [random.choice(x_positions), random.randrange(-10, 10 + 1, 2)]
        margin = 0.8
    elif map == 3:
        start_position = [random.randrange(-5, 5 + 1, 2), -7]
        goal_position = [random.randrange(-5, 5 + 1, 2), 7]
    else:
        start_position = [0, 0]
        distance = 8
        goal_position = [distance*np.cos(random.uniform(0, 2*np.pi)), distance*np.sin(random.uniform(0, 2*np.pi))]
    other_robots = True

    # Init for collision avoidance @Godert Notten
    radius = 0.2
    robot_list = []

    # Settings for the running loop
    state = 0
    timestep = 0

    # For final plot
    covered_path = []
    plotting = True

    # Running loop
    run = True
    while run:
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
                trajectory = mainRRT(obstacles, mountPositions[0, 0:2], goal_position, margin)
                trajectory = np.array(trajectory).reshape((len(trajectory), 2))
                # Create the Enviroment

                env, m, currentPositions, currentOrientations, steeringInput = initEnv(mountPositions,
                                                                                       trajectory,
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

                start_time = time.perf_counter()

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

            if timestep > len(trajectory):
                currentVelocities[0], currentAngularVelocities[0] = mainMPC(len(trajectory),
                                                                            currentPositions[0,:].tolist(),
                                                                            currentOrientations[0],
                                                                            trajectory)
            else:
                currentVelocities[0], currentAngularVelocities[0] = mainMPC(timestep,
                                                                            currentPositions[0,:].tolist(),
                                                                            currentOrientations[0],
                                                                            trajectory)

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

            covered_path.append(currentPositions[0].tolist())

            # Check if the final position has been reached
            if np.linalg.norm(np.array([currentPositions[0,:]]) - trajectory[-1]) < 0.5:
                state = 1
                print("We have reached our goal")

                stop_time = time.perf_counter()
                print(f"Run time elapsed: {stop_time - start_time:0.4f} for a simulated time of: {timestep * 0.01} seconds")

                # Turn into array for plotting
                covered_path = np.array(covered_path)

                if plotting:
                    obstacleList = []
                    for i in range(len(obstacles)):
                        obstacle = ObstacleRectangle(
                            obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3], margin)
                        obstacleList.append(obstacle)

                    # Plot
                    plt.figure(figsize=(9, 9))
                    plt.minorticks_on()
                    plt.axis('equal')
                    plt.plot(trajectory[:, 0], trajectory[:, 1], linestyle="--", color="gray", label="Trajectory")
                    plt.plot(covered_path[:, 0], covered_path[:, 1], "red", label="Our path")
                    plt.plot(start_position[0], start_position[1], color='green', marker='s', markersize=10, label="Start")
                    plt.plot(goal_position[0], goal_position[1], color='green', marker='*', markersize=10, label="Goal")
                    for robot in robot_list:
                        robot.draw()
                    for obs in obstacleList:
                        if obs.type == 'rectangle':
                            plt.gca().add_patch(
                                plt.Rectangle((obs.x1, obs.y1), obs.width, obs.height, fc='darkred', ec='darkred'))
                    redBox = Patch(color='darkred', label='Obstacle')
                    grayPath = Patch(color='gray', label='Trajectory')
                    redPath = Patch(color='red', label='Our path')
                    blueRobot = Patch(color='navy', label='Robots')
                    royalblue = Patch(color='royalblue', label='Our robot')
                    greenStart = Patch(color='green', label='Start/Goal')
                    handles = [redBox, grayPath, redPath, blueRobot, royalblue, greenStart]
                    plt.legend(handles=handles, bbox_to_anchor=(1, 1), borderaxespad=0.)
                    plt.show(block=False)
                    plt.pause(10)

                run = False

            timestep += 1

        # This state signifies that we have finished
        if state == 1:
            continue


behaviour()
