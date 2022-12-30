import numpy as np
import matplotlib.pyplot as plt

class Robot():

    def __init__(self, x, y, r, v_x, v_y, theta, our):
        self.x = x
        self.y = y
        self.r = r
        self.desired_vx = np.cos(theta) * 1
        self.desired_vy = np.sin(theta) * 1
        self.previous_vx = v_x
        self.previous_vy = v_y
        self.output_vx = v_x
        self.output_vy = v_y
        self.theta = theta
        self.our = our

    def move(self, dt):
        self.x += self.output_vx * dt
        self.y += self.output_vy * dt

    def draw(self, plt):
        circle = plt.Circle((self.x, self.y), self.r)
        plt.gca().add_patch(circle)

        if self.our:
            circle = plt.Circle((self.x + self.output_vx, self.y + self.output_vy), 0.1, color='red')
            plt.gca().add_patch(circle)
