import matplotlib.pyplot as plt
import numpy as np

def errorFunction(t,dt,  currentState , path, o):
    """
    Retruns the error matrix A, B for the error dynamics

    path input must be [x,y,vx,vy]

    curentState
    """

    # Velocities of the path
    currentPostionPath = path[t ,:2]
    previousPositionPath = path[t - 1 ,:2]
    previousPreviousPositionPath = path[t - 2 ,:2]

    previousVelocityPathX = ((previousPositionPath[0] - previousPreviousPositionPath[0])*dt)
    previousVelocityPathY = ((previousPositionPath[1] - previousPreviousPositionPath[1])*dt)  
    previousVelocityPath = ((previousVelocityPathX**2 + previousVelocityPathY**2)**0.5)

    velocityPathX = ((currentPostionPath[0] -previousPositionPath[0])*dt)
    velocityPathY = ((currentPostionPath[1] -previousPositionPath[1])*dt)  
    velocityPath = (velocityPathX**2 + velocityPathY**2)**0.5
    anglePath = np.arctan2( velocityPathY ,velocityPathX)

    # Acceleration of the path
    accX = (velocityPathX - previousVelocityPathX)*dt
    accY = (velocityPathY - previousVelocityPathY)*dt

    # Angular acceleration 
    angularVelocityPath = (velocityPathX * accY - velocityPathY*accX)/(velocityPathX**2 + velocityPathY**2 )
    currentAngleBot = o
    
    # angularVelocityBot = (previousAngleBot - currentAngleBot)*dt  
    xError = path[t,0] - currentState[len(currentState)-1,0]
    yError = (path[t,1] - currentState[len(currentState)-1,1] )

    thetaError = anglePath - o


    if thetaError < -6.28:
        thetaError = thetaError - 2*np.pi

    if -6.28 <thetaError < -2:
        thetaError = 0.0375

    errorAmatrix = np.array([[1,dt*angularVelocityPath,0],
                            [-dt*angularVelocityPath, 1 , dt*velocityPath],
                            [0,0,1]])

    errorBmatrix = np.array([[-dt, 0],
                            [0,0],
                            [0,-dt]])

    currentError = np.array([[xError*np.cos(currentAngleBot) + yError * np.sin(currentAngleBot)],
                            [-xError * np.sin(currentAngleBot) + yError * np.cos(currentAngleBot)],
                            [thetaError]])

    errorUmatrix = np.array([velocityPath*np.cos(thetaError),
                                angularVelocityPath])

    return (errorAmatrix, currentError,  errorBmatrix)

# State placeholders
states = []
inputs = []



def mainMPC(t, currentPostion = None, currentOrtientation = None, trajectory = None): 
    """
    Returns the desired velocity and angular velocity 

    INPUT
    timestep -> int : 0
    currentPosition -> np.array() : [x,y,theta] : shape (3,1)
    currentOrientation -> np.float : 0.0
    trajectory -> np.array() : shape -> (n,2)

    OUTPUT
    currentVelocities[0] -> np.float: 0.0
    angularVelocity -> np.float: 0.0
    """

    testing = False
    global states
    global inputs
    dt = 0.1

   
    # Check if we are in testing mode
    if (trajectory is None and currentOrtientation is None) and currentPostion is None: 
        testing = True
    else:
        # Reformatting 
        currentPostion = np.array([currentPostion[0],currentPostion[1],currentOrtientation]).reshape((3,1))



    # Sanity check for testing mode
    if not testing: 
        assert currentPostion.any() != None,      "Kolff MPC is expecting a currentPostion reveiced None"
        assert currentOrtientation != None,       "Kolff MPC is expecting a currentOrientation reveiced None"
        assert trajectory.any() != None,          "Kolff MPC is expecting a trajectory reveiced None"

        assert currentPostion.shape == (3,1),                    f"Kollf MPC requires currentpostion must be of shape (3,1) recieved: {currentPostion.shape}"
        assert isinstance(currentOrtientation, np.floating),     f"Kollf MPC requires currentOrtientation must be of type numpy.float64()"
        assert trajectory.shape[0] > 3,                      f"MPC requires more than three points for the trajectory reveiced {trajectory.shape[0]}"
        assert trajectory.shape[1] == 2,                     f"MPC requires 2 positional arguments for trajectory received {trajectory.shape[0]}"

    
    # Initialize test variables
    if testing: 
        
        uni = UniCycleModel(dt)
        dummyDataX = np.arange(1 ,101 ,0.01)
        dummyDataY = np.sin(dummyDataX)
        target = np.vstack((dummyDataX,dummyDataY)).T   # Test trajectory
        timestep = 0                                    # Current time
        lengthRange = 100 
    else: 
        windowOffset = 3 
        target = trajectory[t - windowOffset : t + 10 - windowOffset ,:]    # Find reference window using time step
        timestep = windowOffset                                             # Middel of window
        lengthRange = 1 
    
    for i in range(lengthRange):          
        if testing:
            o = uni.X[2][0]   
        else:
            o = np.float64(currentOrtientation)

        if len(states) > 3:
            """
            The error model needs to differentiate the path of the current time step and the previous timestep
            therfore we need at least three points in the state array
            """
            error = errorFunction(timestep - 2 ,dt, np.array(states) , target, o)
            input   = mpcControl(error, 10, np.array(states)[timestep - 2,:], target[timestep - 2, :])[0]
            inputs.append(input) 
        else: 
            input = np.array([0,0])
        
        # Check if main is passing arguments otherwise use testdata
        if testing:
            currentState = uni.nextX(input.reshape((1,2)))
            states.append([currentState.tolist()[0][0], currentState.tolist()[1][0]])
            timestep += 1 
            print(f"this is the currentstate: {o} ")
        else: 
            currentState = currentPostion
            states.append(currentState)
            return input[0], input[1]


    # if testing:
    #     fig, ax = plt.subplots()
    #     plot(ax,  np.array(states), target )

def runTest(target,dt): 
    uni = UniCycleModel(dt)
    input = np.array([[0,0]])

    for i in range(0,int(np.pi/dt)):
        pos = uni.nextX(input.reshape((1,2)))[:2].reshape((1,2))
        currentTimePos = [pos[0][0],pos[0][1]]

        if uni.X[2][0]  > 2*np.pi: 
            uni.X[2][0]  = uni.X[2][0] - 2*np.pi
        orientation = uni.X[2][0]

        input = mainMPC(i, currentTimePos, orientation, target)
        input = np.array(list(input))

    return (np.array(states), target )

def test():
    dts = [0.05]

    routes = []

    for dt in dts:
        print(f"this is dt: {dt}")
        dummyDatag = np.arange(0 , np.pi ,dt)

        dummyDataY = np.sin(dummyDatag)
        dummyDataX = -(dummyDatag) 

        dummyDataY = (dummyDatag)
        dummyDataX = np.zeros(len(dummyDataY))

        target1 = np.vstack((dummyDataX,dummyDataY)).T   # Test trajectory

        # Segment 1
        DataX1 = np.arange(0 , 0.5*np.pi ,dt)
        DataY1 = np.linspace(0 , 1 ,len(DataX1))
        placeholder1 = np.vstack((DataX1,DataY1))

        # Segement 2
        DataX2 = np.arange(0.5*np.pi , np.pi ,dt)
        DataY2 = np.linspace(1 , 0 ,len(DataX2))
        placeholder2 = np.vstack((DataX2,DataY2))
        
        target2 = np.hstack((placeholder1, placeholder2)).T

        targets = [target1,target2]

        # for target in targets:
        #     input, target = runTest(target1, dt)
        #     routes.append((input,target))



        input, target = runTest(target1, dt)
        routes.append((input,target))


    fig, ax = plt.subplots()
    ax.plot(input[:,0],input[:,1])
    ax.plot(target[:,0],target[:,1])

    # fig, axs = plt.subplots(2, 2)
    # axs[0, 0].plot(routes[0][0][:,0], routes[0][0][:,1])
    # axs[0, 0].plot(routes[0][1][:,0], routes[0][1][:,1])
    # axs[0, 0].set_title('Axis [0, 0]')
    # axs[1, 0].plot(routes[1][0][:,0], routes[1][0][:,1])
    # axs[1, 0].plot(routes[1][1][:,0], routes[1][1][:,1])

    # axs[0, 1].plot(routes[2][0][:,0], routes[2][0][:,1])
    # axs[0, 1].plot(routes[2][1][:,0], routes[2][1][:,1])
    # axs[0, 1].set_title('Axis [0, 1]')
    # axs[1, 1].plot(routes[3][0][:,0], routes[3][0][:,1])
    # axs[1, 1].plot(routes[3][1][:,0], routes[3][1][:,1])

    # for ax in axs.flat:
    #     ax.set(xlabel='x-pos [m]', ylabel='y-pos [m]')

    # # Hide x labels and tick labels for top plots and y ticks for right plots.
    # for ax in axs.flat:
    #     ax.label_outer()

    plt.show()


if __name__ == '__main__':
    print("main")
    from uni_cycle_model import UniCycleModel 
    from controller import mpcControl, PID
    from testers import plot, testerMPC, testerUni
    test()
else:
    from model_predictive_control.uni_cycle_model import UniCycleModel 
    from model_predictive_control.controller import mpcControl, PID
    from model_predictive_control.testers import plot, testerMPC, testerUni
