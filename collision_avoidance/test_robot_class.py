# Importing own files
from model_predictive_control.MPC import mainMPC
from collision_avoidance.robot_class import Robot
from collision_avoidance.uni_cycle_test import UniCycleModel
from collision_avoidance.test_cases import cases

# Importing libraries
import numpy as np
import matplotlib.pyplot as plt


def testCollisionAvoidance():
    """"
    Start the enviroment and run the algorithms
    Input -> None : None
    Ouput -> None : None
    """""

    # Initialization of all robots positions, velocities and orientations
    test_case = 2
    n_robots = 5
    dt = 0.1

    currentPositions, currentVelocities, currentOrientations, angularVelocity, trajectory = cases(test_case, n_robots)

    # Init for MPC
    uni_list = []
    for i in range(n_robots):
        uni_list.append(UniCycleModel(dt,
                                      currentPositions[i, 0],
                                      currentPositions[i, 1],
                                      currentOrientations[i]))

    # Init for collision avoidance
    radius = 0.2
    robot_list = []
    # Init robots list
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
        robot_list[i].dt = dt

    # Settings for the running loop
    previousPositions = []
    angularVelocities = []

    # Running loop
    for timestep in range(len(trajectory)):

        # MPC
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

        currentVelocities[0], angularVelocity = mainMPC(timestep,
                                                        currentPositions[0, :].tolist(),
                                                        currentOrientations[0],
                                                        trajectory)

        # Collision avoidance
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
                # Can be changed to show plotting, plots are saved for reference
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
                angularVelocities.append(angularVelocity)

        # Update the position and orientation of the robot
        for index, uni in enumerate(uni_list):
            if index == 0:
                newPosition = np.array([currentVelocities[index], angularVelocity])
                xytheta = uni.nextX(newPosition.reshape((1, 2)))
                xy = xytheta[:2]
                previousPositions.append(xy.reshape(1, 2))
                currentPositions[index, :] = xy.flatten()
                currentOrientations[index] = xytheta[2]
            else:
                newPosition = np.array([currentVelocities[index], 0])
                xytheta = uni.nextX(newPosition.reshape((1, 2)))
                xy = xytheta[:2]
                currentPositions[index, :] = xy.flatten()
                currentOrientations[index] = xytheta[2]


    plt.cla()
    plt.minorticks_on()
    plt.axis('equal')
    if test_case == 1:
        plt.xlim(-2, 8)
        plt.ylim(-1, 8)
    if test_case == 2:
        plt.xlim(0, 10)
        plt.ylim(0, 10)
    for robot in robot_list:
        robot.draw(plt)
    plt.plot(np.sum(np.array(previousPositions), axis=1)[:, 0], np.sum(np.array(previousPositions), axis=1)[:, 1])
    plt.scatter(np.sum(np.array(previousPositions), axis=1)[::1, 0], np.sum(np.array(previousPositions), axis=1)[::1, 1], s=100, edgecolors='r', facecolors='blue')

    plt.show()


testCollisionAvoidance()