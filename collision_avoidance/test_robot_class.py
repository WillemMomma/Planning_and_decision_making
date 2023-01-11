# Importing own files
from model_predictive_control.MPC import mainMPC
from collision_avoidance.robot_class import Robot as RobotGVO
from collision_avoidance.robot_class_velocity_obstacles import Robot as RobotVO
from collision_avoidance.uni_cycle_test import UniCycleModel
from collision_avoidance.test_cases import cases

# Importing libraries
import numpy as np
import time
import matplotlib.pyplot as plt


def testCollisionAvoidance(plotter, data, GVO):
    """"
    Start the enviroment and run the algorithms
    Input -> None : None
    Ouput -> None : None
    """""

    # Initialization of all robots positions, velocities and orientations
    test_case = 2
    n_robots = 5
    dt = 0.05

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
    if GVO:
        for i in range(len(currentPositions)):
            if i == 0:
                robot_list.append(RobotGVO(currentPositions[i, 0],
                                        currentPositions[i, 1],
                                        radius,
                                        currentVelocities[i],
                                        angularVelocity,
                                        currentOrientations[i],
                                        True))
            else:
                robot_list.append(RobotGVO(currentPositions[i, 0],
                                        currentPositions[i, 1],
                                        radius,
                                        currentVelocities[i],
                                        0,
                                        currentOrientations[i],
                                        False))
            robot_list[i].dt = dt
    else:
        for i in range(len(currentPositions)):
            if i == 0:
                robot_list.append(RobotVO(currentPositions[i, 0],
                                        currentPositions[i, 1],
                                        radius,
                                        currentVelocities[i],
                                        angularVelocity,
                                        currentOrientations[i],
                                        True))
            else:
                robot_list.append(RobotVO(currentPositions[i, 0],
                                        currentPositions[i, 1],
                                        radius,
                                        currentVelocities[i],
                                        0,
                                        currentOrientations[i],
                                        False))
            robot_list[i].dt = dt

    if test_case == 4:
        if n_robots > 1:
            robot_list[1].output_w = 2
        if n_robots > 2:
            robot_list[2].output_w = -2

    # For data collection
    averageVelocity = 0
    averageDistance = 0
    collision = False
    start = time.perf_counter()

    # Settings for the running loop
    previousPositions = []
    otherPositions = np.zeros((n_robots, 2, len(trajectory)))
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
                                           robot_list[i].output_w,
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
                newPosition = np.array([currentVelocities[index], robot_list[index].output_w])
                xytheta = uni.nextX(newPosition.reshape((1, 2)))
                xy = xytheta[:2]
                otherPositions[index, :, timestep] =xy.reshape(1, 2)
                currentPositions[index, :] = xy.flatten()
                currentOrientations[index] = xytheta[2]

        if data:
            averageVelocity += currentVelocities[0]
            dist = np.linalg.norm(currentPositions[0] - trajectory[timestep])
            averageDistance += dist
            for index, currentPosition in enumerate(currentPositions):
                if index != 0:
                    if np.linalg.norm(currentPosition - currentPositions[0]) < (robot_list[index].r + robot_list[0].r):
                        collision = True

    stop = time.perf_counter()

    if plotter:
        fig = plt.figure()
        plt.cla()
        plt.minorticks_on()
        plt.axis('equal')
        if test_case == 1:
            plt.xlim(-2, 8)
            plt.ylim(-2, 8)
        if test_case == 2:
            plt.xlim(0, 10)
            plt.ylim(0, 10)
        # for robot in robot_list:
        #     robot.draw()
        step = 1
        plt.scatter(np.sum(np.array(previousPositions), axis=1)[::1, 0][::step],
                    np.sum(np.array(previousPositions), axis=1)[::1, 1][::step], s=150, edgecolors='navy', facecolors='blue')
        for i in range(n_robots):
            if i != 0:
                plt.scatter(otherPositions[i, 0][::step], otherPositions[i, 1][::step], s=150, edgecolors='darkred', facecolors='red')

        plt.show()

    averageVelocity /= len(trajectory)
    averageDistance /= len(trajectory)
    total_time = stop-start
    return averageVelocity, averageDistance, collision, total_time

plotter = True
data = True
GVO = True

averageVelocity, averageDistance, collision, total_time = testCollisionAvoidance(plotter, data, GVO)

if data:
    print(averageVelocity, averageDistance, collision, total_time)