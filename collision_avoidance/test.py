from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np
import random
from robot_class import Robot

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
    sample_range = 1
    v = np.array([our_robot.desired_vx, our_robot.desired_vy])

    found = False

    # Start sampling random velocities and keep one which is closest to desired velocity
    for i in range(20):
        vx = random.uniform(v[0] - sample_range, v[0] + sample_range)
        vy = random.uniform(v[1] - sample_range, v[1] + sample_range)
        sampled_velocity = Point(vx, vy)

        if not collision_check(sampled_velocity, cones):
            dist = np.linalg.norm(v - np.array([vx, vy]))
            if dist < closest_distance:
                closest_distance = dist
                new_velocity = [vx, vy]
                found = True

    if found:
        our_robot.output_vx = new_velocity[0]
        our_robot.output_vy = new_velocity[1]


our_robot = Robot(0, 0, 1, 0.5, 0.5, True)
point = Point(0.5, 0.5)
polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
cones = [polygon]

collision = collision_check(point, cones)
print(collision)

resolve(our_robot, cones)
print(our_robot.desired_vx, our_robot.desired_vy)
print(our_robot.output_vx, our_robot.output_vy)
