# Import libraries
import numpy as np
import random
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon

# Import from other .py files
from collision_avoidance.helper import cart2polar, polar2cart


class Robot:

    def __init__(self, x, y, r, v, w, theta, our):

        self.dt = 0.1
        self.plotting = False
        self.plot_number = 0

        # Init position and radius of robot
        self.x = x
        self.y = y
        self.r = r

        # Input given to the robot at initialization
        self.input_v = v
        self.input_vx = np.cos(theta + w*self.dt) * v
        self.input_vy = np.sin(theta + w*self.dt) * v
        self.input_w = w

        # Set previous and output equal to input for the first timestep
        self.previous_v = self.input_v
        self.previous_vx = self.input_vx
        self.previous_vy = self.input_vy
        self.previous_w = self.input_w

        self.output_v = self.input_v
        self.output_vx = self.input_vx
        self.output_vy = self.input_vy
        self.output_w = self.input_w

        # Init orientation and if it is the to-be-controlled robot
        self.theta = theta
        self.our = our

    def update_other(self, x, y, v, w, theta):

        # Init position and radius of robot
        self.x = x
        self.y = y

        # Input given to the robot at initialization
        self.input_v = v
        self.input_w = w
        self.input_vx = np.cos(theta + w*self.dt) * v
        self.input_vy = np.sin(theta + w*self.dt) * v

        # Init orientation and if it is the to-be-controlled robot
        self.theta = theta

    def update_our(self, x, y, v, w, theta, robot_list):

        # Update information incoming from the main
        self.x = x
        self.y = y

        self.input_v = v
        self.input_w = w
        self.input_vx = np.cos(theta + w*self.dt) * v
        self.input_vy = np.sin(theta + w*self.dt) * v

        self.theta = theta

        # Plot the current velocity obstacles
        if self.plotting:
            # Setup plot
            fig = plt.figure()
            plt.cla()
            plt.minorticks_on()
            plt.axis('equal')
            plt.xlim(0, 5)
            plt.ylim(-3, 3)

        # Detect all velocity obstacles here
        cones = self.detect(robot_list)

        # Check for collision with any of the obstacles for desired velocity
        input_velocity = Point(self.x + self.input_vx, self.y + self.input_vy)
        previous_velocity = Point(self.x + self.previous_vx, self.y + self.previous_vy)

        # If no collision is found with desired velocity, continue with desired velocity
        if not self.collision_check(input_velocity, cones):
            self.output_v = self.input_v
            self.output_vx = self.input_vx
            self.output_vy = self.input_vy
            self.output_w = self.input_w

        # If collision is found with desired velocity, check for previous velocity
        elif not self.collision_check(previous_velocity, cones):
            self.output_v = self.previous_v
            self.output_vx = self.previous_vx
            self.output_vy = self.previous_vy
            self.output_w = self.previous_w

        # Else sample a new velocity
        else:
            self.resolve(cones)

        # Save current output for next timestep
        self.previous_v = self.output_v
        self.previous_vx = self.output_vx
        self.previous_vy = self.output_vy
        self.previous_w = self.output_w

        if self.plotting:
            # Draw and move the robots for this timestep
            for robot in robot_list:
                robot.draw(plt)

            # Show and close plot
            plt.show(block=False)
            plt.pause(0.1)
            plt.savefig(f'plotnumber_{self.plot_number}')
            self.plot_number += 1
            plt.close(fig)

    def detect(self, robot_list):

        # Hyper parameters
        cone_size = 1000
        safety_factor = 1.5
        threshold_distance = 10

        # Init our robot and cones list
        cones = []

        # Loop trough complete robot_list
        for robot in robot_list:

            # For all robots that are not ours
            if not robot.our:

                # Calculate relative distance to robot
                p_rel = [robot.x - self.x, robot.y - self.y]

                # Direction and distance between two robots
                dist, angle = cart2polar(p_rel)

                if dist < threshold_distance:

                    # Calculate the angle of the velocity obstacle and calculate points to draw cone
                    cone_angle = np.arcsin((robot.r + self.r) / dist) * safety_factor
                    cone_distance = ((robot.r + self.r) / np.tan(cone_angle)) * cone_size
                    p1 = polar2cart(cone_distance, angle - cone_angle)
                    p2 = polar2cart(cone_distance, angle + cone_angle)

                    # Offset the cone with velocity of other robot and plot the cone and get three points for triangle
                    if dist < 0.75:
                        center = [self.x + robot.input_vx - p_rel[0]*dist, self.y + robot.input_vy - p_rel[1]*dist]
                    else:
                        center = [self.x + robot.input_vx, self.y + robot.input_vy]
                    p1 += center
                    p2 += center

                    if self.plotting:
                        # Plot the cone
                        plt.plot([center[0], p1[0]], [center[1], p1[1]], 'bo', linestyle="--", color="grey")
                        plt.plot([center[0], p2[0]], [center[1], p2[1]], 'bo', linestyle="--", color="grey")

                    # Append all the cones to a list for collision detection
                    cones.append(Polygon([(center[0], center[1]), (p1[0], p1[1]), (p2[0], p2[1])]))

        return cones

    def resolve(self, cones):

        # Set closest sampled point distance and range within to sample for new velocities
        closest_distance = 1e6
        new_velocity = [0, 0]

        # Range in which to sample for new velocities
        min_velocity = 0
        max_velocity = 1.5
        max_w = 3

        # Desired velocity without angular velocity
        v_input = np.array([self.input_vx + self.x, self.input_vy + self.y])

        plt_resolve = False
        found = False

        # Start sampling random velocities and keep one which is closest to desired velocity
        while not found:

            # Try 10 times to find a new velocity
            for i in range(10):

                # Sample velocity in more strictly taken range with angle and min/max vel
                sampledVelocity = random.uniform(min_velocity, max_velocity)
                sampledAngularVelocity = random.uniform(-max_w, max_w)
                # sampledAngle = self.theta + sampledAngularVelocity * self.dt
                vx = np.cos(self.theta + sampledAngularVelocity * self.dt) * sampledVelocity
                vy = np.sin(self.theta + sampledAngularVelocity * self.dt) * sampledVelocity
                # vx, vy = polar2cart(sampledVelocity, sampledAngle)

                # Create a point from the sampled velocity
                velocityPoint = Point(vx + self.x, vy + self.y)

                # Plot the sampled velocity
                if plt_resolve:
                    fig = plt.figure()
                    plt.gca()
                    plt.minorticks_on()
                    plt.axis('equal')
                    plt.xlim(0, 5)

                    for cone in cones:
                        plt.plot(*cone.exterior.xy)

                    plt.scatter(velocityPoint.x, velocityPoint.y)

                    # Show and close plot
                    plt.show(block=False)
                    plt.pause(0.1)
                    plt.close(fig)

                # If no collision occurs for the newly sampled velocity
                if not self.collision_check(velocityPoint, cones):

                    # Toggle found to True
                    found = True

                    # Calc distance to desired velocity
                    dist = np.linalg.norm(v_input - np.array([vx, vy])) + 100*sampledAngularVelocity

                    # If newly sampled value is closest to desired velocity, keep this one
                    if dist < closest_distance:
                        closest_distance = dist
                        new_velocity = [sampledVelocity, sampledAngularVelocity]

            max_w += 1

        # # Set output of our robot to newly sampled velocity
        self.output_v = new_velocity[0]
        self.output_w = new_velocity[1]

    @staticmethod
    def collision_check(velocity, cones):

        # Set collision to False and start check
        collision = False

        # Check if our desired velocity is in any of the velocity obstacles
        for cone in cones:
            if cone.contains(velocity):
                collision = True

        return collision

    def move(self, dt):
        self.x += self.output_vx * dt
        self.y += self.output_vy * dt

    def draw(self, plt):
        circle = plt.Circle((self.x, self.y), self.r)
        plt.gca().add_patch(circle)

        if self.our:
            circle = plt.Circle((self.x + self.output_vx, self.y + self.output_vy), 0.1, color='red')
            plt.gca().add_patch(circle)
