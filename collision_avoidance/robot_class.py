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
        """""
        Input: x -> float
        y -> float
        v -> float
        w -> float
        theta -> float

        Output: None but all the variables of self are updated in this function
        """""

        # Init position and radius of robot
        self.x = x
        self.y = y

        # Input given to the robot at update
        self.input_v = v
        self.input_w = w
        self.input_vx = np.cos(theta + w*self.dt) * v
        self.input_vy = np.sin(theta + w*self.dt) * v

        # Set previous and output equal to input
        self.previous_v = self.input_v
        self.previous_w = self.input_w
        self.previous_vx = self.input_vx
        self.previous_vy = self.input_vy

        self.output_v = self.input_v
        self.output_w = self.input_w
        self.output_vx = self.input_vx
        self.output_vy = self.input_vy

        # Init orientation and if it is the to-be-controlled robot
        self.theta = theta

    def update_our(self, x, y, v, w, theta, robot_list):
        """""
        Input: x -> float
        y -> float
        v -> float
        w -> float
        theta -> float
        robot_list -> list filled with Robot objects

        Output: None but all the variables of self are updated in this function
        """""

        # Update information incoming from the main
        self.x = x
        self.y = y
        self.theta = theta
        self.input_v = v
        self.input_w = w
        self.input_vx = np.cos(theta + w*self.dt) * v
        self.input_vy = np.sin(theta + w*self.dt) * v

        # Plot the current velocity obstacles
        if self.plotting:
            # Setup plot
            fig = plt.figure()
            plt.cla()
            plt.minorticks_on()
            plt.axis('equal')
            plt.xlim(0, 10)
            plt.ylim(0, 10)

        # Detect all velocity obstacles here
        cones = self.detect(robot_list)

        # Check for collision with any of the obstacles for desired velocity
        input_velocity = Point(self.x + self.input_vx, self.y + self.input_vy)
        previous_velocity = Point(self.x + self.previous_vx, self.y + self.previous_vy)
        half_half = Point((self.x + self.input_vx + self.x + self.previous_vx)/2,
                          (self.y + self.input_vy + self.y + self.previous_vy)/2)

        # If no collision is found with desired velocity, continue with desired velocity
        if not self.collision_check(input_velocity, cones):
            self.output_v = self.input_v
            self.output_vx = self.input_vx
            self.output_vy = self.input_vy
            self.output_w = self.input_w

        elif not self.collision_check(half_half, cones):
            self.output_v = self.previous_v
            self.output_vx = self.previous_vx
            self.output_vy = self.previous_vy
            self.output_w = self.previous_w

        # If collision is found with desired velocity, check for previous velocity
        elif not self.collision_check(previous_velocity, cones) and self.previous_v != 0:
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
            # plt.show(block=False)
            # plt.pause(0.1)
            plt.savefig(f'plotnumber_{self.plot_number}')
            self.plot_number += 1
            plt.close(fig)

    def detect(self, robot_list):
        """""
        Input: robot_list -> list filled with Robot objects
        
        Output: cones -> list filled with shapely.geometry.Polygon objects 
                        which resemble the collision areas in velocity space
        """""

        # Hyper parameters
        cone_size = 1000
        safety_factor = 1.8
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

                # Only check for robots within threshold_distance
                if dist < threshold_distance:

                    # Calculate the angle of the velocity obstacle and calculate points to draw cone
                    cone_angle = np.arcsin((robot.r + self.r) / dist) * safety_factor
                    cone_distance = ((robot.r + self.r) / np.tan(cone_angle)) * cone_size
                    p1 = polar2cart(cone_distance, angle - cone_angle)
                    p2 = polar2cart(cone_distance, angle + cone_angle)

                    # Move collision free velocity space away from obstacle for extra safety
                    if dist < 1:
                        scaler = (1/dist - 1)
                        center = [self.x + robot.input_vx - p_rel[0]*scaler, self.y + robot.input_vy - p_rel[1]*scaler]
                        # center = [self.x + robot.input_vx, self.y + robot.input_vy]
                    else:
                        center = [self.x + robot.input_vx, self.y + robot.input_vy]
                    # Offset the cone with velocity of other robot and plot the cone and get three points for triangle
                    p1 += center
                    p2 += center

                    # For plotting
                    if self.plotting:
                        # Plot the cone
                        plt.plot([center[0], p1[0]], [center[1], p1[1]], linestyle="--", color="grey")
                        plt.plot([center[0], p2[0]], [center[1], p2[1]], linestyle="--", color="grey")

                    # Append all the cones to a list for collision detection
                    cones.append(Polygon([(center[0], center[1]), (p1[0], p1[1]), (p2[0], p2[1])]))

        return cones

    def resolve(self, cones):
        """""
        Input: cones -> list filled with shapely.geometry.Polygon objects 
                        which resemble the collision areas in velocity space

        Output: None
        
        Function finds a new velocity for our robot which is collision free in velocity space
        """""

        # Set closest sampled point distance and range within to sample for new velocities
        closest_distance = 1e6
        new_velocity = [0, 0]

        # Range in which to sample for new velocities
        min_velocity = -0.1
        max_velocity = 1.5
        max_w = 1

        # Desired velocity without angular velocity
        v_input = np.array([self.input_vx + self.x, self.input_vy + self.y])
        v_previous = np.array([self.previous_vx + self.x, self.previous_vy + self.y])
        v_input = (v_input*0.75 + v_previous*0.25)

        # Plot and found toggle
        plt_resolve = False
        found = False

        # While no collsion free velocity is found, keep sampling
        while not found:

            # Init plot and plot cone if plt_resolve
            if plt_resolve:
                fig = plt.figure()
                plt.gca()
                plt.minorticks_on()
                plt.axis('equal')
                plt.xlim(0, 5)
                for cone in cones:
                    plt.plot(*cone.exterior.xy)

            # Try 10 times to find a new velocity
            for i in range(40):

                # Sample velocity in more strictly taken range with angle and min/max vel
                sampledVelocity = random.uniform(min_velocity, max_velocity)
                sampledAngularVelocity = random.uniform(-max_w, max_w)
                vx = np.cos(self.theta + sampledAngularVelocity * self.dt) * sampledVelocity
                vy = np.sin(self.theta + sampledAngularVelocity * self.dt) * sampledVelocity

                # Create a point from the sampled velocity
                velocityPoint = Point(vx + self.x, vy + self.y)

                # Plot the sampled velocity
                if plt_resolve:
                    plt.scatter(velocityPoint.x, velocityPoint.y)

                # If no collision occurs for the newly sampled velocity
                if not self.collision_check(velocityPoint, cones):

                    # Toggle found to True
                    found = True

                    # Calc distance to desired velocity with a weight to a high angular velocity
                    dist = np.linalg.norm(v_input - np.array([vx, vy])) + sampledAngularVelocity

                    # If newly sampled value is closest to desired velocity, keep this one
                    if dist < closest_distance:
                        closest_distance = dist
                        new_velocity = [sampledVelocity, sampledAngularVelocity]

            # Show and close plot
            if plt_resolve:
                # Show and close plot
                plt.show(block=False)
                plt.pause(0.1)
                plt.close(fig)

            # If no collision free velocity is found, increase the search area
            max_w += 0.5

        # # Set output of our robot to newly sampled velocity
        self.output_v = new_velocity[0]
        self.output_w = new_velocity[1]
        self.output_vx = np.cos(self.theta + self.output_w * self.dt) * self.output_v
        self.output_vy = np.sin(self.theta + self.output_w * self.dt) * self.output_v

    @staticmethod
    def collision_check(velocity, cones):
        """""
        Input: velocity -> shapely.geometry.Point object 
        cones -> list filled with shapely.geometry.Polygon objects which resemble the collision areas in velocity space

        Output: collision -> Boolean

        Function checks if a velocity is collsion free in velocity space
        """""

        # Set collision to False and start check
        collision = False

        # Check if our desired velocity is in any of the velocity obstacles
        for cone in cones:
            if cone.contains(velocity):
                collision = True

        return collision

    def draw(self, plt):
        """""
        Input: plt -> Plot from matplotlib 

        Output: None

        Plots the robots in a plot
        """""
        circle = plt.Circle((self.x, self.y), self.r)
        plt.gca().add_patch(circle)

        if self.our:
            plt.title(f"v: {np.round(self.output_v, 2)}, w: {np.round(self.output_w, 2)}, vx: {np.round(self.output_vx, 2)}, vy: {np.round(self.output_vy, 2)}")
            circle = plt.Circle((self.x + np.cos(self.theta), self.y + np.sin(self.theta)), 0.05, color='green')
            plt.gca().add_patch(circle)
            circle = plt.Circle((self.x + self.output_vx, self.y + self.output_vy), 0.1, color='red')
            plt.gca().add_patch(circle)
