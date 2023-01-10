from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningEnv.urdfObstacle import UrdfObstacle
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle
import numpy as np

import os


#Walls
poseStorage  = [

              [-1, -4, 0],
              [-3, -4, 0],
              [-5, -4, 0],
              [-7, -4, 0],
              [-9, -4, 0],
              [-1, 4, 0],
              [-3, 4, 0],
              [-5, 4, 0],
              [-7, 4, 0],
              [-9, 4, 0],
              [1, 4, 0],              
              [1, -4, 0],              
              [3, -4, 0],
              [5, -4, 0],
              [7, -4, 0],
              [9, -4, 0],
              [3, 4, 0],
              [5, 4, 0],
              [7, 4, 0],
              [9, 4, 0]

        ]

poseWall1  = [
          [-7.75, 1.25, 0]

    ]

poseWall2  = [
          [-4, 7.1, np.pi * 0.5]

    ]
    
poseWallboxes  = [
          [-3, -1.5, 0],
          [-6, -1.5, 0],
          [-3, -3, 0],
          [-6, -3, 0],

    ]

    
walls1 = [["GEOM_BOX", [0.4, 6, 0.25], poseStorage],["GEOM_BOX", [0.2, 11.5, 1], poseWall1],["GEOM_BOX", [0.2, 8.35, 1], poseWall2]]

walls1 = [["GEOM_BOX", [0.4, 6, 0.25], poseStorage]]




#Boxes
#shape_type="GEOM_BOX", dim=[0.2, 0.2, 0.2],mass = 0.3, poses_2d=poseWallboxes , place_height = 0.9

boxes1 = [["GEOM_BOX", [0.4, 0.4, 0.4], 0.3, poseWallboxes, 0.5],["GEOM_BOX", [0.3, 0.3, 0.2], 0.3, poseWallboxes, 0.9],["GEOM_BOX", [0.2, 0.2, 0.3], 0.3, poseWallboxes, 1.5]]



#Obstacles:
obst1Dict = {
    "type": "sphere",
    "geometry": {"position": [2.0, 2.0, 1.0], "radius": 1.0},
}
sphereObst1 = SphereObstacle(name="simpleSphere", content_dict=obst1Dict)
obst2Dict = {
    "type": "sphere",
    'movable': True,
    "geometry": {"position": [2.0, -0.0, 0.5], "radius": 0.2},
}
sphereObst2 = SphereObstacle(name="simpleSphere", content_dict=obst2Dict)
urdfObst1Dict = {
    'type': 'urdf',
    'geometry': {'position': [1.5, 0.0, 0.05]},
    'urdf': os.path.join(os.path.dirname(__file__), 'obstacle_data/duck.urdf'),
}
urdfObst1 = UrdfObstacle(name='duckUrdf', content_dict=urdfObst1Dict)
dynamicObst1Dict = {
    "type": "sphere",
    "geometry": {"trajectory": ['2.0 - 0.1 * t', '-0.0', '0.1'], "radius": 0.2},
}
dynamicSphereObst1 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst1Dict)
dynamicObst2Dict = {
    "type": "analyticSphere",
    "geometry": {"trajectory": [' 2', '2 * t ', '0'], "radius": 0.2},
}
dynamicSphereObst2 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst2Dict)


splineDict = {'degree': 2, 'controlPoints': [[0.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 0.0]], 'duration': 10}
dynamicObst3Dict = {
    "type": "splineSphere",
    "geometry": {"trajectory": splineDict, "radius": 0.2},
}
dynamicSphereObst3 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst3Dict)

dynamicObst4Dict = {
    "type": "analyticSphere",
    "geometry": {"trajectory": ['3', 't * 2.5 ', '0'], "radius": 0.2},
}
dynamicSphereObst4 = DynamicSphereObstacle(name="simpleSphere", content_dict=dynamicObst4Dict)



obstacles1 = [dynamicSphereObst2, dynamicSphereObst4]


