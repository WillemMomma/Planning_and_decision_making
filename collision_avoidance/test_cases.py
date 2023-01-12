import numpy as np

def cases(case_index, n_robots):

    currentPositions = None
    currentVelocities = None
    currentOrientations = None
    angularVelocity = None
    trajectory = None

    # Number 1
    if case_index == 1:

        # Position
        currentPositions = np.ones((n_robots, 2))
        # Velocity
        currentVelocities = np.zeros((n_robots,))
        # Orientation
        currentOrientations = np.zeros((n_robots,))
        # Angular velocity
        angularVelocity = np.float64(0)

        if n_robots > 1:
            currentPositions[1, :] = [3, 2]
            currentVelocities[1] = 2
            currentOrientations[1] = -np.pi / 2

        if n_robots > 2:
            currentPositions[2, :] = [6, 3]
            currentVelocities[2] = 0
            currentOrientations[2] = -np.pi / 2

        if n_robots > 3:
            currentPositions[3, :] = [-5, 7]
            currentVelocities[3] = 2
            currentOrientations[3] = 0

        # Trajectory
        steps = 30
        low = 1
        high = 5
        dummyDataX = np.linspace(low, high, steps)
        dummyDataX = np.append(dummyDataX, np.linspace(high, high, steps)[1:])
        dummyDataX = np.append(dummyDataX, np.linspace(high, -high, steps * 2)[1:])
        # dummyDataX = np.append(dummyDataX, np.linspace(low, low, steps)[1:])
        dummyDataY = np.linspace(low, low, steps)
        dummyDataY = np.append(dummyDataY, np.linspace(low, high, steps)[1:])
        dummyDataY = np.append(dummyDataY, np.linspace(high, high, steps * 2)[1:])
        # dummyDataY = np.append(dummyDataY, np.linspace(high, low, steps)[1:])
        trajectory = np.vstack((dummyDataX, dummyDataY)).T  # Test trajectory

    # Number 2
    if case_index == 2:

        # Position
        currentPositions = np.ones((n_robots, 2))
        if n_robots > 1:
            currentPositions[1, :] = [3, 1]
        if n_robots > 2:
            currentPositions[2, :] = [5, 1.5]
        if n_robots > 3:
            currentPositions[3, :] = [7, 1]
        if n_robots > 4:
            currentPositions[4, :] = [9, 0.5]
        currentPositions[:, 1] += 4
        # Velocity
        currentVelocities = np.zeros((n_robots,))
        # Orientation
        currentOrientations = np.zeros((n_robots,))
        # Angular velocity
        angularVelocity = np.float64(0)

        # Trajectory
        steps = 80
        dummyDataX = np.linspace(1, 20, steps)
        dummyDataY = np.linspace(5, 5, steps)
        trajectory = np.vstack((dummyDataX, dummyDataY)).T  # Test trajectory

    # Number 3
    if case_index == 3:

        # Velocity
        currentVelocities = np.zeros((n_robots,))
        # Orientation
        currentOrientations = np.zeros((n_robots,))
        # Angular velocity
        angularVelocity = np.float64(0)
        # Position
        currentPositions = np.ones((n_robots, 2))
        if n_robots > 1:
            currentPositions[1, :] = [15, 2]
            currentOrientations[1] = np.pi
            currentVelocities[1] = 2
        if n_robots > 2:
            currentPositions[2, :] = [10, 1]
            currentOrientations[2] = np.pi
            currentVelocities[2] = 2
        if n_robots > 3:
            currentPositions[3, :] = [5, 0.8]
            currentOrientations[3] = np.pi
            currentVelocities[3] = 2
        currentPositions[:, 1] += 4

        # Trajectory
        steps = 80
        dummyDataX = np.linspace(1, 20, steps)
        dummyDataY = np.linspace(5, 5, steps)
        trajectory = np.vstack((dummyDataX, dummyDataY)).T  # Test trajectory

    # Number 4
    if case_index == 4:

        # Velocity
        currentVelocities = np.zeros((n_robots,))
        # Orientation
        currentOrientations = np.zeros((n_robots,))
        # Angular velocity
        angularVelocity = np.float64(0)
        # Position
        currentPositions = np.ones((n_robots, 2))
        if n_robots > 1:
            currentPositions[1, :] = [9, 1]
            currentOrientations[1] = np.pi/2
            currentVelocities[1] = 3
        if n_robots > 2:
            currentPositions[2, :] = [5, 1]
            currentOrientations[2] = np.pi/2
            currentVelocities[2] = 3
        currentPositions[:, 1] += 4

        # Trajectory
        steps = 80
        dummyDataX = np.linspace(1, 20, steps)
        dummyDataY = np.linspace(5, 5, steps)
        trajectory = np.vstack((dummyDataX, dummyDataY)).T  # Test trajectory


    return currentPositions, currentVelocities, currentOrientations, angularVelocity, trajectory