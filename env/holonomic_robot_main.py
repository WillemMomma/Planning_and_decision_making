import gym
import numpy as np
import itertools
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle


def walls2obstacles(wall):
    result = []
    for i in range(len(wall)):
        dimensions = wall[i][1][0:2]
        coordinates = [coord[:2] for coord in wall[i][2]]

        result.append([[coordinates[0] - 0.5 * dimensions[0], coordinates[1] - 0.5 * dimensions[1],
                        coordinates[0] + 0.5 * dimensions[0], coordinates[1] + 0.5 * dimensions[1]] for coordinates
                    in coordinates])
    obstacles = list(itertools.chain(*result))
    return obstacles

def rotationMatrix(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

def initMap(otherrobots, maps, start_position):
    if maps == 0:
        mountPositions = np.array([(start_position[0], start_position[1], 0)])
        obstacles = ([(100, 100, 100, 100)])

    elif maps == 1 or maps == 2:
        if otherrobots == True and maps == 1:
            mountPositions = np.array(
                [
                    (start_position[0], start_position[1], 0), (6, -8, 0), (-6, 8, 0), (4, 0.5, 0), (-6, -0.5, 0), (-12, -0.5, 0), (12, 0.5, 0)
                ]
            )
        if otherrobots == True and maps == 2:
            mountPositions = np.array(
                [
                    (start_position[0], start_position[1], 0), (5, -8, 0), (-3, 8, 0), (3, 0.5, 0), (-5, -0.5, 0), (-11, -0.5, 0), (11, 0.5, 0)
                ]
            )


            #make an array with the start position of the robots with constant value of x = 7 but different y starting with 2 till 12 with a step of 2 in a loop
            # similair to this (start_position[0],start_position[1],0),(-7,1,0),(-7,3,0),(-7,5,0),(-7,7,0),(-7,9,0),(-7,11,0)


        result = []
        from env.gym_envs_urdf.scene_objects.obstacles import walls1, walls2
        if maps == 1:
            obstacles = walls2obstacles(walls1)
        elif maps == 2:
            obstacles = walls2obstacles(walls2)

    if maps == 3:



        mountPositions = np.array(
        [
            (start_position[0],start_position[1],0),(-7,-4,0),(-7,-2,0),(-7,0,0),(-7,2,0),(-7,4,0),(-7,6,0),
                                                        (7,-5,0),(7,-3,0),(7,-1,0),(7,1,0),(7,3,0),(7,5,0),
                                                        (-7.5,-4,0),(-7.5,-2,0),(-7.5,0,0),(-7.5,2,0),(-7.5,4,0),(-7.5,6,0),
                                                        (7.5,-5,0),(7.5,-3,0),(7.5,-1,0),(7.5,1,0),(7.5,3,0),(7.5,5,0)
        ]
    )
        obstacles = ([(100, 100, 100, 100)])

    return mountPositions, obstacles


def initEnv(mountPositions, trajectory, otherrobots=False, maps=0, dt=0.01):
    try:
        x1 = trajectory[0, 0]
        y1 = trajectory[0, 1]
        x2 = trajectory[1, 0]
        y2 = trajectory[1, 1]
        angle = np.arctan2(y2 - y1, x2 - x1)
    except:
        print("No trajectory found")
        angle = 0

    class steering:
        def straight(v, n):
            vector = np.array([v, 0, 0])
            m = int(n / v * (1 / dt))
            arr = np.tile(vector, (m, 1))
            return (arr)

        def left(v, r, deg):
            omega = v / r
            m = int((np.pi * (1 / dt) * deg) / (180 * omega))
            vector = np.array([v, 0, omega])
            arr = np.tile(vector, (m, 1))
            return (arr)

        def right(v, r, deg):
            omega = v / r
            m = int((np.pi * (1 / dt) * deg) / (180 * omega))
            vector = np.array([v, 0, -omega])
            arr = np.tile(vector, (m, 1))
            return (arr)

        def stop(t):
            vector = np.array([0, 0, 0])
            m = int((1 / dt) * t)
            arr = np.tile(vector, (m, 1))
            return (arr)

    if maps == 0 or otherrobots == False:
        robots = [
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel")
        ]
        m = len(robots)
        env = gym.make(
            "urdf-env-v0",
            dt=dt, robots=robots, render=True
        )
        mountPositions = mountPositions
        initialPositions = np.array(
            [
                (0, 0, angle - np.deg2rad(90))
            ]
        )
        initialPositions[:, :2] = mountPositions[:, :2]
        steeringInput = np.zeros((3000))
        env.reset(pos=initialPositions, mount_positions=mountPositions)

    if (maps == 1 or maps == 2) and otherrobots == True:
        robots = [
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel")
        ]
        m = len(robots)
        env = gym.make(
            "urdf-env-v0",
            dt=dt, robots=robots, render = True
        )

        initialPositions = np.array(
            [
                (0, 0, angle - np.deg2rad(90)), (0, 0, 0), (0, 0, np.pi), (0, 0, np.deg2rad(90)),
                                                (0, 0, np.deg2rad(270)), (0, 0, np.deg2rad(270)), (0, 0, np.deg2rad(90))
        
            ]
        )

        mountPositions = mountPositions

        env.reset(pos=initialPositions, mount_positions=mountPositions)

    if maps ==3: 

        arr1 = np.concatenate(
            [steering.straight(6, 6.5), steering.left(6, 1, 92), steering.straight(6, 9), steering.right(6, 2, 91),
             steering.straight(6, 0.1), steering.left(6, 2, 25), steering.right(6, 2, 25), steering.straight(6, 2),
             steering.right(6, 2, 25), steering.left(6, 2, 25), steering.left(6, 2, 180), steering.straight(6, 14),
             steering.left(6, 1, 91), steering.straight(6, 10), steering.left(6, 1, 91), steering.straight(6, 6),
             steering.right(6, 2, 91), steering.straight(6, 3)])
        arr2 = np.concatenate(
            [steering.straight(6, 6.5), steering.left(6, 1, 92), steering.straight(6, 6), steering.right(6, 2, 91),
             steering.straight(6, 5.5), steering.left(6, 2, 180), steering.straight(6, 14), steering.left(6, 1, 91),
             steering.straight(6, 10), steering.left(6, 1, 91), steering.straight(6, 6), steering.right(6, 2, 91),
             steering.straight(6, 3)])
        arr3 = np.concatenate(
            [steering.straight(6, 9), steering.right(6, 1, 91), steering.straight(6, 5.5), steering.left(6, 2, 180),
             steering.straight(6, 2), steering.left(6, 2, 25), steering.right(6, 2, 25), steering.straight(6, 2),
             steering.right(6, 2, 25), steering.left(6, 2, 25), steering.straight(4, 7), steering.left(6, 1, 91),
             steering.straight(6, 10), steering.left(6, 1, 91), steering.straight(6, 6), steering.right(6, 2, 91),
             steering.straight(6, 3)])
        arr4 = np.concatenate(
            [steering.straight(6, 5), steering.right(6, 1, 91), steering.straight(6, 5.5), steering.left(6, 2, 182),
             steering.straight(6, 2), steering.left(6, 2, 25), steering.right(6, 2, 25), steering.straight(6, 2),
             steering.right(6, 2, 25), steering.left(6, 2, 25), steering.straight(4, 6), steering.left(6, 1, 91),
             steering.straight(6, 7), steering.left(6, 1, 91), steering.straight(6, 6), steering.right(6, 2, 91),
             steering.straight(6, 3)])
        arr5 = np.concatenate([steering.stop(3), steering.straight(4, 26)])
        arr6 = np.concatenate([steering.stop(4), steering.straight(4, 26)])

        max_len = max(arr1.shape[0], arr2.shape[0], arr3.shape[0], arr4.shape[0], arr5.shape[0], arr6.shape[0])

        arr1 = np.pad(arr1, [(0, max_len - arr1.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr2 = np.pad(arr2, [(0, max_len - arr2.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr3 = np.pad(arr3, [(0, max_len - arr3.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr4 = np.pad(arr4, [(0, max_len - arr4.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr5 = np.pad(arr5, [(0, max_len - arr5.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr6 = np.pad(arr6, [(0, max_len - arr6.shape[0]), (0, 0)], 'constant', constant_values=0)

        steeringInput = np.concatenate([arr1, arr2, arr3, arr4, arr5, arr6], axis=1)

    # I want to make a new map with a lot of robots no obstacles. Using the steering class 
    # I can make a new map with a lot of robots and obstacles. But I want to make a new map with a lot of robots and no obstacles.
    if maps == 3:
        #initialize robots
        robots = [
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel"),
            GenericUrdfReacher(urdf="pointRobot.urdf", mode="vel")
        ]
        m = len(robots)
        env = gym.make(
            "urdf-env-v0",
            dt=dt, robots=robots, render = True
        )
        
        initialPositions = np.array(
            [
                (0, 0, angle - np.deg2rad(90)), (0, 0, np.deg2rad(270)), (0, 0, np.deg2rad(270)),(0, 0, np.deg2rad(270)), (0, 0, np.deg2rad(270)),(0, 0, np.deg2rad(270)), (0, 0, np.deg2rad(270)),
                                                (0, 0, np.deg2rad(90)), (0, 0, np.deg2rad(90)), (0, 0, np.deg2rad(90)), (0, 0, np.deg2rad(90)),(0, 0, np.deg2rad(90)), (0, 0, np.deg2rad(90)),
                                                (0, 0, np.deg2rad(270)), (0, 0, np.deg2rad(270)),(0, 0, np.deg2rad(270)), (0, 0, np.deg2rad(270)),(0, 0, np.deg2rad(270)), (0, 0, np.deg2rad(270)),
                                                (0, 0, np.deg2rad(90)), (0, 0, np.deg2rad(90)), (0, 0, np.deg2rad(90)), (0, 0, np.deg2rad(90)),(0, 0, np.deg2rad(90)), (0, 0, np.deg2rad(90))
                                                
            ]
        )

        mountPositions = mountPositions

        env.reset(pos=initialPositions, mount_positions=mountPositions)


        arr1 = np.concatenate(
            [steering.stop(0.5),steering.straight(2, 14)]
        )
        arr2 = np.concatenate(
            [steering.stop(0.6),steering.straight(4, 14)]
        )
        arr3 = np.concatenate(
            [steering.stop(1),steering.straight(3, 14)]
        )
        arr4 = np.concatenate(
            [steering.stop(1.5),steering.straight(4, 14)]
        )
        arr5 = np.concatenate(
            [steering.stop(.5),steering.straight(2.5, 14)]
        )
        arr6 = np.concatenate(
            [steering.stop(2),steering.straight(6.5, 14)]
        )
        arr7 = np.concatenate(
            [steering.stop(0.3),steering.straight(4.3, 14)]
        )
        arr8 = np.concatenate(
            [steering.stop(0.6),steering.straight(5.6, 14)]
        )
        arr9 = np.concatenate(
            [steering.stop(1.3),steering.straight(2.5, 14)]
        )
        arr10 = np.concatenate(
            [steering.straight(2, 14)]
        )
        arr11 = np.concatenate(
            [steering.stop(1.7),steering.straight(5.5, 14)]
        )
        arr12 = np.concatenate(
            [steering.stop(0.5),steering.straight(3.8, 14)]
        )
        arr13 = np.concatenate(
            [steering.stop(2.5),steering.straight(3, 14)]
        )
        arr14 = np.concatenate(
            [steering.stop(2.6),steering.straight(5, 14)]
        )
        arr15 = np.concatenate(
            [steering.stop(3),steering.straight(4, 14)]
        )
        arr16 = np.concatenate(
            [steering.stop(3.5),steering.straight(6, 14)]
        )
        arr17 = np.concatenate(
            [steering.stop(2.5),steering.straight(3.5, 14)]
        )
        arr18 = np.concatenate(
            [steering.stop(3.5),steering.straight(7.5, 14)]
        )
        arr19 = np.concatenate(
            [steering.stop(2.5),steering.straight(5.3, 14)]
        )
        arr20 = np.concatenate(
            [steering.stop(1.6),steering.straight(6.6, 14)]
        )
        arr21 = np.concatenate(
            [steering.stop(3.3),steering.straight(3.5, 14)]
        )
        arr22 = np.concatenate(
            [steering.stop(3.3),steering.straight(3, 14)]
        )
        arr23 = np.concatenate(
            [steering.stop(3.7),steering.straight(6.5, 14)]
        )
        arr24 = np.concatenate(
            [steering.stop(2.7),steering.straight(4.8, 14)]
        )


        max_len = max(arr1.shape[0], arr2.shape[0], arr3.shape[0], arr4.shape[0], arr5.shape[0], arr6.shape[0], arr7.shape[0], arr8.shape[0], arr9.shape[0], arr10.shape[0], arr11.shape[0], arr12.shape[0], arr13.shape[0], arr14.shape[0], arr15.shape[0], arr16.shape[0], arr17.shape[0], arr18.shape[0], arr19.shape[0], arr20.shape[0], arr21.shape[0], arr22.shape[0], arr23.shape[0], arr24.shape[0])

        arr1 = np.pad(arr1, [(0, max_len - arr1.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr2 = np.pad(arr2, [(0, max_len - arr2.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr3 = np.pad(arr3, [(0, max_len - arr3.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr4 = np.pad(arr4, [(0, max_len - arr4.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr5 = np.pad(arr5, [(0, max_len - arr5.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr6 = np.pad(arr6, [(0, max_len - arr6.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr7 = np.pad(arr7, [(0, max_len - arr7.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr8 = np.pad(arr8, [(0, max_len - arr8.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr9 = np.pad(arr9, [(0, max_len - arr9.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr10 = np.pad(arr10, [(0, max_len - arr10.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr11 = np.pad(arr11, [(0, max_len - arr11.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr12 = np.pad(arr12, [(0, max_len - arr12.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr13 = np.pad(arr13, [(0, max_len - arr13.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr14 = np.pad(arr14, [(0, max_len - arr14.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr15 = np.pad(arr15, [(0, max_len - arr15.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr16 = np.pad(arr16, [(0, max_len - arr16.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr17 = np.pad(arr17, [(0, max_len - arr17.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr18 = np.pad(arr18, [(0, max_len - arr18.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr19 = np.pad(arr19, [(0, max_len - arr19.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr20 = np.pad(arr20, [(0, max_len - arr20.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr21 = np.pad(arr21, [(0, max_len - arr21.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr22 = np.pad(arr22, [(0, max_len - arr22.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr23 = np.pad(arr23, [(0, max_len - arr23.shape[0]), (0, 0)], 'constant', constant_values=0)
        arr24 = np.pad(arr24, [(0, max_len - arr24.shape[0]), (0, 0)], 'constant', constant_values=0)


        steeringInput = np.concatenate([arr1, arr2, arr3, arr4, arr5, arr6, arr7, arr8, arr9, arr10, arr11, arr12, arr13, arr14, arr15, arr16, arr17, arr18, arr19, arr20, arr21, arr22, arr23, arr24], axis=1)

        



    from env.gym_envs_urdf.scene_objects.obstacles import walls1, walls2

    if maps == 1:
        for wall in walls1:
            env.add_shapes(shape_type=wall[0], dim=wall[1], poses_2d=wall[2])
    elif maps == 2:
         for wall in walls2:
             env.add_shapes(shape_type=wall[0], dim=wall[1], poses_2d=wall[2])

    # n = env.n()
    # pos0 = np.zeros👎
    # pos0[1] = 0.0

    return env, m, mountPositions[:, :2], initialPositions[:, 2] + np.deg2rad(90), steeringInput



def robotMain(mountPositions, m, pos, vel, current_orientations, omega, otherRobots, env, dt=0.01):
    x = pos[0][0]
    y = pos[0][1]
    theta = current_orientations[0]
    # Calculate the velocity vector in the original orientation
    v_vector = np.array([vel, 0])

    # Create a rotation matrix to rotate the velocity vector
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])
    # Rotate the velocity vector
    v_rotated = rotationMatrix(theta) @ v_vector

    # Calculate the new position and orientation
    x_new = x + v_rotated[0] * dt
    y_new = y + v_rotated[1] * dt
    theta_new = theta + omega * dt

    # Create the new position and orientation arrays
    vel_new_oud = np.array([vel])
    vel_rot_new = np.array([omega])
    pos_new_oud = np.array([[x_new, y_new]])
    orientation_new_oud = np.array([theta_new])

    # Create the action array for the current robot
    action_new = np.append(v_rotated, omega)

    if m > 1:
        # Calculate the new position and orientation for the other robots

        result = otherRobots.reshape((m - 1, 3))

        # Extract position and orientation from the inputs
        for i in range(m - 1):
            x = pos[i + 1][0]
            y = pos[i + 1][1]
            theta = current_orientations[i + 1]
            v_vector = result[i][0:2]
            omega = result[i][2]
            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                        [np.sin(theta), np.cos(theta)]])
            # Rotate the velocity vector
            v_rotated = rotation_matrix @ v_vector

            # Calculate the new position and orientation
            x_new = x + v_rotated[0] * dt
            y_new = y + v_rotated[1] * dt
            theta_new = theta + omega * dt

            vel_new_oud = np.append(vel_new_oud, result[i][0])
            vel_rot_new = np.append(vel_rot_new, omega)
            pos_new_oud = np.append(pos_new_oud, x_new)
            pos_new_oud = np.append(pos_new_oud, y_new)
            orientation_new_oud = np.append(orientation_new_oud, theta_new)

            action_new = np.append(action_new, v_rotated)
            action_new = np.append(action_new, omega)

    pos_new_oud = pos_new_oud.reshape((m, 2))

    # Step the environment and return the new position, velocity, and orientation
    ob, _, _, _ = env.step(action_new)
    pos_new = []
    vel_new = []
    orientation_new = []
    vel_rot_new = []
    # print(ob)
    for robot_data in ob.items():
        position = robot_data[1]['joint_state']['position']
        pos_new = np.append(pos_new, position[0:2])
        orientation_new = np.append(orientation_new, position[2])

        vel_rot_new = np.append(vel_rot_new, robot_data[1]['joint_state']['velocity'][2])
        velocity = robot_data[1]['joint_state']['velocity'][0:2]
        velocity = (velocity[0] ** 2 + velocity[1]**2) ** 0.5
        vel_new = np.append(vel_new, velocity)

    # print("Oud",vel)
    # print("NEW",velocity)
    orientation_new = orientation_new + np.deg2rad(90)
    pos_new = pos_new.reshape((m, 2))
    if m > 1:
        pos_new += mountPositions[:, :2]

    return pos_new, vel_rot_new, vel_new, orientation_new