import numpy as np

# Turn cartesian into polar coordinates
def cart2polar(vector):
    mag = np.linalg.norm(vector)
    direction = np.arctan2(vector[1], vector[0])
    return mag, direction

# Turn polar into cartesian coordinates
def polar2cart(mag, direction):
    x = mag * np.cos(direction)
    y = mag * np.sin(direction)
    cart = np.array([x, y])
    return cart

# Turn degree into radian
def deg2radian(angle):
    return angle * np.pi / 180