# Import from libraries
import numpy as np
from matplotlib import pyplot as plt
import random
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

# Import from other .py files
from collision_avoidance.helper import cart2polar, polar2cart, deg2radian


# Cone creator
def detect(robot_list):

    # Hyper parameters
    cone_size = 2
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
            dist, angle = cart2polar(p_rel)

            if dist < threshold_distance:

                # Calculate the angle of the velocity obstacle and calculate points to draw cone
                cone_angle = np.arcsin((robot.r + our_robot.r)/dist) * safety_factor
                cone_distance = ((robot.r + our_robot.r) / np.tan(cone_angle)) * cone_size
                p1 = polar2cart(cone_distance, angle - cone_angle)
                p2 = polar2cart(cone_distance, angle + cone_angle)

                # Offset the cone with velocity of other robot and plot the cone and get thee points for triangle
                center = [our_robot.x + robot.input_vx, our_robot.y + robot.input_vy]
                p1 += center
                p2 += center

                # Plot the cone
                plt.plot([center[0], p1[0]], [center[1], p1[1]], 'bo', linestyle="--", color="grey")
                plt.plot([center[0], p2[0]], [center[1], p2[1]], 'bo', linestyle="--", color="grey")

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

    dt = our_robot.dt

    # Set closest sampled point distance and range within to sample for new velocities
    closest_distance = 1e6
    new_velocity = [0, 0]

    # Range in which to sample for new velocities
    min_velocity = 0
    max_velocity = 3
    max_w = 3
    # max_angle = deg2radian(15)

    # Desired velocity
    v_input = np.array([our_robot.input_vx, our_robot.input_vy])
    #
    # # Direction and magnitude of desired velocity
    # _, desired_angle = cart2polar(v_input)

    # Start sampling random velocities and keep one which is closest to desired velocity
    for i in range(20):

        # Sample velocity in more strictly taken range with angle and min/max vel
        sampledVelocity = random.uniform(min_velocity, max_velocity)
        sampledAngularVelocity = random.uniform(-max_w, max_w)
        sampledAngle = our_robot.theta + sampledAngularVelocity*dt
        vx, vy = polar2cart(sampledVelocity, sampledAngle)

        # Create a point from the sampled velocity
        velocityPoint = Point(vx + our_robot.x, vy + our_robot.y)

        # If no collision occurs for the newly sampled velocity
        if not collision_check(velocityPoint, cones):

            # Calc distance to desired velocity
            dist = np.linalg.norm(v_input - np.array([vx, vy]))

            # If newly sampled value is closest to desired velocity, keep this one
            if dist < closest_distance:
                closest_distance = dist
                new_velocity = [vx, vy]

    # Set output of our robot to newly sampled velocity
    our_robot.output_v = sampledVelocity
    our_robot.output_vx = new_velocity[0]
    our_robot.output_vy = new_velocity[1]
    our_robot.output_w = sampledAngularVelocity
