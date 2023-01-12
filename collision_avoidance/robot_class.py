# Import libraries
import numpy as np
import random
import matplotlib.pyplot as plt

# Import from other .py files
from collision_avoidance.uni_cycle_test import UniCycleModel


class Robot:

    def __init__(self, x, y, r, v, w, theta, our):

        self.dt = 0.01
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

        half_half = np.array([self.input_v + self.previous_v, self.input_w + self.previous_w])/2

        # Only consider the close robots in the list
        min_dist = 10
        close_robot_list = []

        for robot in robot_list:
            dist = np.linalg.norm([robot.x - self.x, robot.y - self.y])
            if dist < min_dist:
                close_robot_list.append(robot)

        # If no collision is found with desired velocity, continue with desired velocity
        if not self.check_validity(close_robot_list, self.input_v, self.input_w):
            self.output_v = self.input_v
            self.output_vx = self.input_vx
            self.output_vy = self.input_vy
            self.output_w = self.input_w

        # If collision is found with desired velocity, check for previous velocity
        elif not self.check_validity(close_robot_list, self.previous_v, self.previous_w) and self.previous_v != 0:
            self.output_v = self.previous_v
            self.output_vx = self.previous_vx
            self.output_vy = self.previous_vy
            self.output_w = self.previous_w

        # If collision is found with desired velocity, check for previous velocity
        elif not self.check_validity(close_robot_list, half_half[0], half_half[1]) and self.previous_v != 0:
            self.output_v = self.previous_v
            self.output_vx = self.previous_vx
            self.output_vy = self.previous_vy
            self.output_w = self.previous_w

        # Else sample a new velocity with gvo
        else:
            self.resolve_gvo(close_robot_list)

        # Save current output for next timestep
        self.previous_v = self.output_v
        self.previous_vx = self.output_vx
        self.previous_vy = self.output_vy
        self.previous_w = self.output_w

        if self.plotting:
            # Draw and move the robots for this timestep
            for robot in robot_list:
                robot.draw()

            # Show and close plot
            # plt.show(block=False)
            # plt.pause(0.1)
            plt.savefig(f'plotnumber_{self.plot_number}')
            self.plot_number += 1
            plt.close(fig)

    def resolve_gvo(self, robot_list):
        """""
        Input: cones -> list filled with shapely.geometry.Polygon objects 
                        which resemble the collision areas in velocity space

        Output: None

        Function finds a new velocity for our robot which is collision free in velocity space
        """""

        # Set closest sampled point distance and range within to sample for new velocities
        closest_distance = float("Inf")
        new_velocity = [0, 0]

        # Range in which to sample for new velocities
        min_velocity = -4
        max_velocity = 6
        max_w = 3
        weight = [1, 1]

        # Plot and found toggle
        found = False

        # While no collsion free velocity is found, keep sampling
        while not found:

            print("HIERRRRRR")

            # Try 40 times to find a new velocity
            for i in range(40):

                # Sample velocity in more strictly taken range with angle and min/max vel
                sampledVelocity = random.uniform(min_velocity, max_velocity)
                sampledAngularVelocity = random.uniform(-max_w, max_w)

                # If no collision occurs for the newly sampled velocity
                if not self.check_validity(robot_list, sampledVelocity, sampledAngularVelocity):

                    # Toggle found to True
                    found = True

                    # Calc distance to desired velocity with a weight to a high angular velocity
                    input = np.array([self.input_v*weight[0], self.input_w*weight[1]])
                    sample = np.array([sampledVelocity*weight[0], sampledAngularVelocity*weight[1]])
                    dist = np.linalg.norm(sample - input)

                    # If newly sampled value is closest to desired velocity, keep this one
                    if dist < closest_distance:
                        closest_distance = dist
                        new_velocity = [sampledVelocity, sampledAngularVelocity]

        # # Set output of our robot to newly sampled velocity
        self.output_v = new_velocity[0]
        self.output_w = new_velocity[1]
        self.output_vx = np.cos(self.theta + self.output_w * self.dt) * self.output_v
        self.output_vy = np.sin(self.theta + self.output_w * self.dt) * self.output_v
        print()

    def check_validity(self, robot_list, testVelocity, testAngulaVelocity):

        # Check for collision 10 seconds into the future
        time_horizon = 5
        safety_margin = 1.5

        # Set collision to False in beginning
        collision = False

        # Toggle for plotting
        plot_gvo = False
        if plot_gvo:
            fig = plt.figure()
            plt.cla()
            plt.minorticks_on()
            plt.axis('equal')
            plt.xlim(0, 10)
            plt.ylim(0, 10)

        # Iterate over robot_list
        for robot in robot_list:

            # Init our positions and other positions
            our_positions = []
            other_positions = []

            # Init two unicycle objects
            our_uni = UniCycleModel(self.dt, self.x, self.y, self.theta)
            other_uni = UniCycleModel(robot.dt, robot.x, robot.y, robot.theta)

            # Update positions for time horizon and check closest distance
            if not robot.our:
                steps = int(time_horizon//self.dt)
                for i in range(steps):
                    ourNewPosition = np.array([testVelocity, testAngulaVelocity])
                    xytheta = our_uni.nextX(ourNewPosition.reshape((1, 2)))
                    xy = xytheta[:2]
                    our_positions.append(xy.reshape(1, 2))

                    otherNewPosition = np.array([robot.output_v, robot.output_w])
                    xytheta = other_uni.nextX(otherNewPosition.reshape((1, 2)))
                    xy = xytheta[:2]
                    other_positions.append(xy.reshape(1, 2))

                # Check min distance per robot
                min_distance = float("inf")
                for index in range(len(other_positions)):
                    distance = np.linalg.norm(np.array(other_positions[index]) - np.array(our_positions[index]))
                    min_distance = min(min_distance, distance)

                # If distance is less than the radii added --> collisison = True
                if min_distance < (self.r + robot.r)*safety_margin:
                    collision = True

                # Plot
                if plot_gvo:
                    plt.scatter(np.sum(np.array(our_positions), axis=1)[:, 0],
                        np.sum(np.array(our_positions), axis=1)[:, 1])
                    plt.scatter(np.sum(np.array(other_positions), axis=1)[:, 0],
                        np.sum(np.array(other_positions), axis=1)[:, 1])

        if plot_gvo:
            plt.show(block=False)
            plt.pause(0.1)
            plt.close(fig)

        return collision

    def draw(self):
        """""
        Input: plt -> Plot from matplotlib 

        Output: None

        Plots the robots in a plot
        """""
        circle = plt.Circle((self.x, self.y), self.r)
        plt.gca().add_patch(circle)

        if self.our:
            plt.title(f"Test case   with   robots")
            circle = plt.Circle((self.x + np.cos(self.theta), self.y + np.sin(self.theta)), 0.05, color='green')
            plt.gca().add_patch(circle)