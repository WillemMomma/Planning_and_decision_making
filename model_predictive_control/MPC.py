import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm

class UniCycleModel:
    """
    Defining the vehicle dynamics as an LTV
    """

    def __init__(self, dt):
        # Euler discretization
        self.A = np.eye(3)
        self.X = np.array([[1],[1],[0]])
        self.B = np.array([ [np.cos(self.X[2])[0],0],
                            [0,np.sin(self.X[2])[0]],
                            [0,1]])

        self.dt = dt

        
    def nextX(self, u):
        """
        Iterate over to the next x
        
        x -> current state  : np.array
        u -> input : np.array
        return -> next state : np.array
        """

        # Updating the arrays
        self.X = self.A.dot(self.X) + self.B.dot(u.T) * self.dt
        self.B = np.array([ [np.cos(self.X[2])[0],0],
                            [np.sin(self.X[2])[0],0],
                            [0,1]])
        return self.X

def mpcControl(error, N, xInit, xTarget):
    """
    Control the vehicle using MPC it is assumed that the operater can only alter the acceleration
    
\
    Normally we would have an LTI, but now we have an LTV therefore
    x[t+1] = A*x + B*u
    
    The state of the bot   ->  [x, y, theta] 
    X represents the error -> [xError, yError, thereError]
    U represents the input -> [v,theta]

    A end B are error matrices packaged in the error variable

    """
    print("we are in MPC")
    
    weightInput = np.array([[1,0],[0,1]])    # Weight on the input
    weightTracking = np.array([[100,0,0],[0,100,0],[0,0,10]]) # Weight on the tracking state
    
    cost = 0.
    constraints = []
    
    # Create the optimization variables
    x = cp.Variable((3, N + 1)) # cp.Variable((dim_1, dim_2))
    u = cp.Variable((2, N))


    # Initialize the current error
    constraints += [x[:, 0] == error[1].reshape((3,))]  
        
    for k in range(N):
        
        nextError = error[0].reshape((3,3))@x[:,k] + error[2].reshape((3,2))@u[:, k]

        # constraints
        constraints += [x[:, k+1] == nextError]
        constraints += [u[:, k] <= [3,3]]
        constraints += [u[:, k] >= [-3,-3]]

        # Minimize the cost function
        cost += cp.quad_form(u[:, k], weightInput)
        cost += cp.quad_form((x[:, k+1] ), weightTracking)

    
    # Solves the problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.OSQP)

    # We return the MPC input and the next state (and also the plan for visualization)
    return u[:, 0].value, x[:, 1].value, x[:, :].value, None

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

    previousVelocityPathX = (previousPositionPath[0] - previousPreviousPositionPath[0])*dt
    previousVelocityPathY = (previousPositionPath[1] - previousPreviousPositionPath[1])*dt  
    previousVelocityPath = ((previousVelocityPathX**2 + previousVelocityPathY**2)**0.5)

    velocityPathX = (currentPostionPath[0] -previousPositionPath[0])*dt
    velocityPathY = (currentPostionPath[1] -previousPositionPath[1])*dt  
    velocityPath = (velocityPathX**2 + velocityPathY**2)**0.5
    anglePath = np.arctan2(velocityPathY , velocityPathX)

    # Acceleration of the path
    accX = (velocityPathX - previousVelocityPathX)*dt
    accY = (velocityPathY - previousVelocityPathY)*dt

    # Angular acceleration 
    angularVelocityPath = (velocityPathX * accY - velocityPathY*accX)/(velocityPathX**2 + velocityPathY**2 )
    currentAngleBot = o
    
    # angularVelocityBot = (previousAngleBot - currentAngleBot)*dt  
    xError = path[t,0] - currentState[len(currentState)-1,0]
    yError = (path[t,1] - currentState[len(currentState)-1,1] )/5
    thetaError = np.arctan2(velocityPathY,velocityPathX) - o

    errorAmatrix = np.array([[1,dt*angularVelocityPath,0],
                            [-dt*angularVelocityPath, 1 , dt*velocityPath],
                            [0,0,1]])

    errorBmatrix = np.array([[-dt, 0],
                            [0,0],
                            [0,-dt]])

    currentError = np.array([[xError*np.cos(currentAngleBot) + yError * np.sin(currentAngleBot)],
                            [-xError * np.sin(currentAngleBot) + yError * np.cos(currentAngleBot)],
                            [thetaError]])


    return (errorAmatrix, currentError,  errorBmatrix)

def Run(t, curentState = False, path = [0], orient = 0 ):
    """
    Start simulating the run of the model and plot the results 
    
    input ->  None
    output-> Plots
    """
    
    # Inintialize variables
    dt = 0.1

    if t < 2:
        return

    error = errorFunction(t,dt, curentState , path, orient)
    input_   = mpcControl(error, 10, curentState[t,:], path[t, :])
    
    return input_

def plot(ax,input, target):
   
    input = input[2:]

    for i in range(len(input)):
        ax.plot(input[:i,0],input[:i,1])
        ax.plot(target[:i,0],target[:i,1])
        # Note that using time.sleep does *not* work here!
        plt.pause(0.3)

def testerMPC():
    dummyDataX = np.arange(0 ,100 ,0.1)
    dummyDataY = np.arange(0 ,10 ,0.01)
    target = np.concatenate((dummyDataX,dummyDataY), axis=0).reshape((100,2))

    fig, ax = plt.subplots()

    timestep = 0 

    inputHistory = []
    for i in range(200):
        input = Run(timestep, target, target )
        inputHistory.append(input)
        timestep += 1 
    
    # Plotting 
    return inputHistory

def testerUni():

    model = UniCycleModel(0.1)
    fig, ax = plt.subplots()
    stateHistory = []
    for i in range(2,35):
        input = np.array([[1,1]])
        state = model.nextX(input)
        print(state)

        stateHistory.append(state)
    stateHistory = (np.array(stateHistory))


    plot(ax,stateHistory,stateHistory )


def mainMPC(t, currentPostion = None, currentOrtientation = None, trajectory = None): 
    """
    Hallo bary
    """


    states = []
    inputs = []
    # intialize vehicle
    uni = UniCycleModel(0.1)

    # Check if main is passing arguments otherwise use testdata
    if currentOrtientation is None: 
        dummyDataX = np.arange(1 ,101 ,0.1)
        dummyDataY = np.sin(dummyDataX)
        target = np.vstack((dummyDataX,dummyDataY)).T
    else: 
        # Get the right window 
        windowOffset = 3 
        target = trajectory[t - windowOffset : t + 10 - windowOffset ,:]
    
    # fig, ax = plt.subplots()

    timestep = 3

    for i in range(1):
         
        # Check if main is passing arguments otherwise use testdata
        if currentOrtientation is None: 
            o = uni.X[2][0]
        else: 
            o = np.float(currentOrtientation)
            
        if len(states) > 3:
            input = Run(timestep - 2 , np.array(states), target, o )
            input = input[0]
            inputs.append(input) 
        else: 
            input = np.array([[0,0]])
        
        # Check if main is passing arguments otherwise use testdata
        if currentPostion is None:
            currentState = uni.nextX(input.reshape((1,2)))
            states.append([currentState.tolist()[0][0], currentState.tolist()[1][0]])
        else: 
            currentState = currentPostion
            states.append(currentState)

        print(f"this is the currentstate: {o} ")
                
        return input[0][0], input[0][1]

    # fig, ax = plt.subplots()
    # plot(ax,  np.array(states), target )



    

