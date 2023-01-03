import numpy as np
import matplotlib.pyplot as plt

class Robot():

    def __init__(self, x, y, r, v, w, theta, our):

        self.dt = 0.25

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

    def update(self, x, y, v, w, theta):

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

    def move(self, dt):
        self.x += self.output_vx * dt
        self.y += self.output_vy * dt

    def draw(self, plt):
        circle = plt.Circle((self.x, self.y), self.r)
        plt.gca().add_patch(circle)

        if self.our:
            circle = plt.Circle((self.x + self.output_vx, self.y + self.output_vy), 0.1, color='red')
            plt.gca().add_patch(circle)
