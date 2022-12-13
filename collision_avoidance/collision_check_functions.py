# Import from libraries
import numpy as np
from matplotlib import pyplot as plt
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

# Import from other .py files
from helper import cart2polar, polar2cart


# Cone creator
def detect(robot_list):

    # Hyper parameters
    cone_factor = 2
    safety_factor = 1.2
    threshold_distance = 10

    # Init our robot and cones list
    our_robot = None
    cones = []

    for robot in robot_list:

        # Save our te be controlled robot
        if robot.our:
            our_robot = robot

        # Make cones for the others
        else:
            # Calculate relative velocity and distance to robot
            # v_rel = [robot.desired_vx - our_robot.desired_vx, robot.desired_vy - our_robot.desired_vy]
            p_rel = [robot.x - our_robot.x, robot.y - our_robot.y]

            # Direction and distance between two robots
            e_dist, angle = cart2polar(p_rel)

            if e_dist < threshold_distance:

                # Calculate the angle of the velocity obstacle and calculate points to draw cone
                d_angle = np.arcsin((robot.r + our_robot.r)/e_dist) * safety_factor
                distance = ((robot.r + our_robot.r) / np.tan(d_angle)) * cone_factor
                p1 = polar2cart(distance, angle - d_angle)
                p2 = polar2cart(distance, angle + d_angle)

                # Offset the cone with velocity of other robot and plot the cone
                center = [our_robot.x + robot.desired_vx, our_robot.y + robot.desired_vy]
                p1 += center
                p2 += center
                plt.plot([center[0], p1[0]], [center[1], p1[1]], 'bo', linestyle="--")
                plt.plot([center[0], p2[0]], [center[1], p2[1]], 'bo', linestyle="--")
                x = [p1[0], p2[0]]
                y = [p1[1], p2[1]]
                plt.scatter(x, y)

                # Append all the cones to a list for collision detection
                cones.append(Polygon([(center[0], center[1]), (p1[0], p1[1]), (p2[0], p2[1])]))

    return cones


# Check for collision
def collision_check(velocity, cones):

    # Set collision to False and start check
    collision = False

    # Check if our desired velocity is in any of the velocity obstacles
    for cone in cones:
        if cone.contains(velocity):
            collision = True

    return collision


# Resolve collision
def resolve(our_robot, cones):

    # Set closest sampled point distance and range within to sample for new velocities
    closest_distance = 1e6
    new_velocity = [0, 0]

    # Range in which to sample for new velocities
    sample_range = 1

    # Desired velocity
    v_desired = np.array([our_robot.desired_vx, our_robot.desired_vy])

    # Start sampling random velocities and keep one which is closest to desired velocity
    for i in range(20):
        # Samples velocity
        vx = random.uniform(v_desired[0] - sample_range, v_desired[0] + sample_range)
        vy = random.uniform(v_desired[1] - sample_range, v_desired[1] + sample_range)
        sampled_velocity = Point(vx, vy)

        # If no collision occurs for the newly sampled velocity
        if not collision_check(sampled_velocity, cones):

            # Calc distance to desired velocity
            dist = np.linalg.norm(v_desired - np.array([vx, vy]))

            # If newly sampled value is closest to desired velocity, keep this one
            if dist < closest_distance:
                closest_distance = dist
                new_velocity = [vx, vy]

    # Set output of our robot to newly sampled velocity
    our_robot.output_vx = new_velocity[0]
    our_robot.output_vy = new_velocity[1]
