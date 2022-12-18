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
        self.X = np.zeros((3,1))
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
                            [0,np.sin(self.X[2])[0]],
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

    print("we are in the mpcController")
    
    weightInput = np.array([[1,0],[0,1]])    # Weight on the input
    weightTracking = np.array([[1,0,0],[0,1,0],[0,0,1]]) # Weight on the tracking state
    
    cost = 0.
    constraints = []
    
    # Create the optimization variables
    x = cp.Variable((3, N + 1)) # cp.Variable((dim_1, dim_2))
    u = cp.Variable((2, N))
    constraints += [x[:, 0] == error[1].reshape((3,))]  
        
    for k in range(N):
        
        nextError = error[0].reshape((3,3))@x[:,k] + error[2].reshape((3,2))@u[:, k]

        # constraints
        constraints += [x[:, k+1] == nextError]

        # Minimize the cost function
        cost += cp.quad_form(u[:, k], weightInput)
        cost += cp.quad_form((x[:, k+1] ), weightTracking)

    
    # Solves the problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.OSQP)

    # We return the MPC input and the next state (and also the plan for visualization)
    return u[:, 0].value, x[:, 1].value, x[:, :].value, None

def errorFunction(t,dt,  currentState , path):
    """
    Retruns the error matrix A, B for the error dynamics

    path input must be [x,y,vx,vy]

    curentState
    """

    # initialize the first point

    currentAnglePath = np.tan(path[t + 1 ,1] - path[t,1]/path[t + 1 ,0] - path[t,0] )
    previousAnglePath = np.tan(path[t ,1] - path[t -1 ,1]/path[t ,0] - path[t - 1 ,0] )
    angularVelocityPath = (currentAnglePath - previousAnglePath)*dt 
    
    currentPostionPath = path[t ,:2]
    previousPositionPath = path[t - 1 ,:2]
    velocityPath = ((currentPostionPath[0] -previousPositionPath[0])**2 - (currentPostionPath[1] -previousPositionPath[1])**2)**0.5

    velocityPath = 1

    currentAngleBot = np.tan(currentState[t + 1 ,1] - currentState[t,1]/currentState[t + 1 ,0] - currentState[t,0] )
    previousAngleBot = np.tan(currentState[t ,1] - currentState[t -1 ,1]/currentState[t ,0] - currentState[t - 1 ,0] )
    angularVelocityBot = (previousAngleBot - currentAngleBot)*dt 

    errorAmatrix = np.array([[1,dt,angularVelocityPath],
                            [-dt*angularVelocityPath, 0 , dt*velocityPath],
                            [0,0,1]])

    errorBmatrix = np.array([[-dt, 0],
                            [0,0],
                            [0,-dt]])

    xError = currentState[t,0] - path[t,0]
    yError = currentState[t,1] - path[t,1]
    thetaError = angularVelocityPath - angularVelocityBot

    currentError = np.array([[xError*np.cos(currentAngleBot) + yError * np.sin(currentAngleBot)],
                            [-xError * np.sin(currentAngleBot) - yError * np.cos(currentAngleBot)],
                            [thetaError]])

    return (errorAmatrix, currentError,  errorBmatrix)

def Run(t, curentState = False, path = [0]):
    """
    Start simulating the run of the model and plot the results 
    
    input ->  None
    output-> Plots
    """
    
    # Inintialize variables
    dt = 0.1

    if t < 2:
        return

    error = errorFunction(t,dt, curentState , path)
    input_   = mpcControl(error, 10, curentState[t,:], path[t, :])
    
    return input_

def plot(ax,input):
   
    input = input[2:]

    for i in range(len(input)):
        ax.plot(input[:i,0],input[:i,1])
        # Note that using time.sleep does *not* work here!
        plt.pause(0.1)

def testerMPC():
    dummyDataX = np.arange(0 ,10 ,0.1)
    dummyDataY = np.arange(0 ,1 ,0.01)
    target = np.concatenate((dummyDataX,dummyDataY), axis=0).reshape((100,2))

    fig, ax = plt.subplots()

    timestep = 0 

    inputHistory = []
    for i in range(40):
        input = Run(timestep, target, target )
        inputHistory.append(input)
        timestep += 1 
    
    # Plotting 
    return inputHistory

def testerUni():

    model = UniCycleModel(0.1)
    input = testerMPC()
    fig, ax = plt.subplots()
    stateHistory = []
    for i in range(2,35):
        # input = np.array([[1,2]])
        state = model.nextX(input[i][0])
        print(input[i][0])

        stateHistory.append(state)
    stateHistory = (np.array(stateHistory))


    plot(ax,stateHistory )

# testerUni()

