# Import from libraries
from matplotlib import pyplot as plt
from shapely.geometry import Point
import numpy as np

# Import from other .py files

from collision_avoidance.collision_check_functions import detect, collision_check, resolve
from collision_avoidance.robot_class import Robot

# Will update this branch to take into account vehicle dynamics

def mainCollisionAvoidance(robot_list=None):


    # If no input is given fill the robots list with the given data
    if robot_list is None:
        # Setting 1
        robot1 = Robot(0, 0, 0.2, 0.5, 0, np.pi/2, True)
        robot2 = Robot(-10, 10, 0.2, 0.5, 0, 0, False)
        robot3 = Robot(3, 5, 0.2, -0.5, 0, 0, False)
        robot4 = Robot(10, 15, 0.2, 0.5, 0, 0, False)
        robot_list = [robot1, robot2, robot3, robot4]

    # Setup plot
    fig = plt.figure()
    plt.minorticks_on()
    plt.axis('equal')
    plt.xlim(-15, 15)
    plt.ylim(-15, 15)
    plt.cla()

    # Get the robot we're controlling from the whole list
    our_robot = None
    for robot in robot_list:
        if robot.our:
            our_robot = robot
            break

    # Set timestep
    dt = our_robot.dt

    # Detect all velocity obstacles here
    cones = detect(robot_list)

    # Check for collision with any of the obstacles for desired velocity
    input_velocity = Point(our_robot.x + our_robot.input_vx, our_robot.y + our_robot.input_vy)
    previous_velocity = Point(our_robot.x + our_robot.previous_vx, our_robot.y + our_robot.previous_vy)

    # If no collision is found with desired velocity, continue with desired velocity
    if not collision_check(input_velocity, cones):
        our_robot.output_vx = our_robot.input_vx
        our_robot.output_vy = our_robot.input_vy

    # If collision is found with desired velocity, check for previous velocity
    elif not collision_check(previous_velocity, cones):
        our_robot.output_vx = our_robot.previous_vx
        our_robot.output_vy = our_robot.previous_vy

    # Else sample a new velocity
    else:
        resolve(our_robot, cones)

    # Save current output for next timestep
    our_robot.previous_v = our_robot.output_v
    our_robot.previous_vx = our_robot.output_vx
    our_robot.previous_vy = our_robot.output_vy
    our_robot.previous_w = our_robot.output_w

    print(our_robot.output_vx)

    # Draw and move the robots for this timestep
    for robot in robot_list:
        robot.draw(plt)
        # robot.move(dt)

    # Show and close plot
    plt.show(block=False)
    plt.pause(0.1)
    plt.close(fig)

    return robot_list

# for i in range(200):
#     if i == 0:
#         robot_list = mainCollisionAvoidance()
#     else:
#         for index in len(range(robot_list))
#         robot_list = mainCollisionAvoidance(robot_list)
#
#         robot_list[0].input_v