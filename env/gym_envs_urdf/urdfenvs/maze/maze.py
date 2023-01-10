# -*- coding: utf-8 -*-
"""
Created on Tue Dec 13 15:22:47 2022

@author: wille
"""
import numpy as np



def add_maze(
    self,
    dim=np.array([0.2, 8, 2]),
    poses_2d=None,
) -> None:
    """
    Adds walls to the simulation environment.

    Parameters
    ----------

    dim = [width, length, height]
    poses_2d = [[x_position, y_position, orientation], ...]
    """
    if poses_2d is None:
        poses_2d = [
            [-4, 0.1, 0],
            [4, -0.1, 0],
            [0.1, 4, 0.5 * np.pi],
            [-0.1, -4, 0.5 * np.pi],
        ]
    self.add_shapes(
        shape_type="GEOM_BOX", dim=dim, mass=0, poses_2d=poses_2d
    )
    