# Import from libraries
from matplotlib import pyplot as plt
from shapely.geometry import Point
import numpy as np

# Import from other .py files

from collision_avoidance.collision_check_functions import detect, collision_check, resolve
from collision_avoidance.robot_class import Robot

def mainCollisionAvoidance(positions = None, velocities = None, angularVelocities = None, orientations = None):

    

    # If an input is given fill the robots list with the given data
    if positions is not None:
        
        # Fill robots list
        radius = 1

        # Fill robot list with given values
        robot_list = []
        for i in range(positions.shape[0]):
            if i == 0:
                robot_list.append(Robot(positions[i, 0], positions[i, 1], radius, velocities[i, 0], velocities[i, 1], orientations[i], True))
            else:
                robot_list.append(Robot(positions[i, 0], positions[i, 1], radius, velocities[i, 0], velocities[i, 1], orientations[i], False))

    else:
        # Setting 1
        robot1 = Robot(0, 0, 1, 0, 0.5, True)
        robot2 = Robot(-10, 10, 1, 0.5, 0, False)
        robot3 = Robot(3, 5, 1, 0, 0, False)
        robot4 = Robot(10, 15, 1, -0.5, 0, False)
        robot_list = [robot1, robot2, robot3, robot4]

    # # Setup plot
    # fig = plt.figure()
    # plt.minorticks_on()
    # plt.axis('equal')
    # plt.xlim(-15, 15)
    # plt.ylim(-15, 15)

    # Get the robot we're controlling from the whole list
    our_robot = None
    for robot in robot_list:
        if robot.our:
            our_robot = robot

    # Main loop
    T = 40
    dt = 0.25
    N = int(T/dt)
    for t in range(N):

        # # Clear plot from last iteration
        # plt.cla()

        # Detect all velocity obstacles here
        cones = detect(robot_list)

        # Check for collision with any of the obstacles for desired velocity
        desired_velocity = Point(our_robot.x + our_robot.desired_vx, our_robot.y + our_robot.desired_vy)
        previous_velocity = Point(our_robot.x + our_robot.previous_vx, our_robot.y + our_robot.previous_vy)

        # If no collision is found with desired velocity, continue with desired velocity
        if not collision_check(desired_velocity, cones):
            our_robot.output_vx = our_robot.desired_vx
            our_robot.output_vy = our_robot.desired_vy

        # If collision is found with desired velocity, check for previous velocity
        elif not collision_check(previous_velocity, cones):
            our_robot.output_vx = our_robot.previous_vx
            our_robot.output_vy = our_robot.previous_vy

        # Else sample a new velocity
        else:
            resolve(our_robot, cones)

        # # Save current output for next timestep
        # our_robot.previous_vx = our_robot.output_vx
        # our_robot.previous_vy = our_robot.output_vy

        # Draw and move the robots for this timestep
        # for robot in robot_list:
            # robot.draw(plt)
            # robot.move(dt)

        # plt.plot()
        # plt.pause(0.001)

    # plt.show()

    return np.array([our_robot.output_vx, our_robot.output_vy])
