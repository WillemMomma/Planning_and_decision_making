import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm


class vehicleDynamicsJap:
    """
    Defining the vehicle dynamics, on unicycle model is assumed which is capable of instant acceleration
    """

    def __init__(self, dt):

        self.aC = np.array([
            [0, 1,0,0],
            [0.,0,0,0],
            [0.,0,0,1],
            [0.,0,0,0]])

        self.bC = np.array([
            [0],
            [1],
            [0],
            [1]])

        self.cC = np.array([
            [1, 0]])

        self.dC = np.array([
            [0.]])
        
    
        # Euler discretization
        self.A = np.eye(4) + self.aC * dt
        self.B = self.bC * dt
        self.C = self.cC
        self.D = self.dC

        
    def nextX(self, x, u):
        """
        Iterate over to the next x
        
        x -> current state  : np.array
        u -> input : np.array
        return -> next state : np.array
        """
        return self.A.dot(x) + self.B.dot(u.T).diagonal()
    
    
def mpcControl(vehicle, N, xInit, xTarget):
    """
    Control the vehicle using MPC it is assumed that the operater can only alter the acceleration
    
    vehicle -> model of the dynamics : object vehicleDynamicsJap
    N -> look ahead horizon : Int
    xInit -> current state : np.array
    xTarget -> desired state : np.array
    returns -> next input : list    
    """
    
    weightInput = np.array([[0,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])    # Weight on the input
    weightTracking = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) # Weight on the tracking state
    
    cost = 0.
    constraints = []
    
    # Create the optimization variables
    x = cp.Variable((4, N + 1)) # cp.Variable((dim_1, dim_2))
    u = cp.Variable((4, N))
    
    print(f"this is xinit: {xInit} and this is the shape: {xInit.shape}")
    
    constraints += [x[:, 0] == xInit]
        
    for k in range(N):
        """
        Normally we would have an LTI, but now we have an LTV therefore
        x[t+1] = A*x + B*u*dt
        
        The state of the bot is [x, y, xvel, yvel, theta] 
        Therefore A is the identity matrix 
        u = [x , y, xvel, yvel, (xvel - yvel) ]

        B = [0,0,1,1,0] <- for this we can also use simple contraints
        """
        
        state_ = vehicle.A@x[:,k]
        input_ = u[:, k]
        
        # constraints
        constraints += [x[:, k+1] == state_ + input_]
        constraints += [u[0, k] == 0]
        constraints += [u[2, k] == 0]

        # Minimize the cost function
        cost += cp.quad_form(u[:, k], weightInput)
        cost += cp.quad_form((xTarget - x[:, k+1] ), weightTracking)


    
    # Solves the problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.OSQP)

    # We return the MPC input and the next state (and also the plan for visualization)
    return u[:, 0].value, x[:, 1].value, x[:, :].value, None

def errorFunction(t, curentState , path):
    """
    This function calculates the error of the robot
    """

    headingError, postionError, velocityError = [0,0,0]

    return headingError, postionError, velocityError

def Run(t, curentState = False, path = [0]):
    """
    Start simulating the run of the model and plot the results 
    
    input ->  None
    output-> Plots
    """

    # headingError, postionError, velocityError = errorFunction(t, curentState , path)

    # Inintialize variables
    dt = 0.1
    start = 0
    stop = 10

    lookahead = 10

    if t == 0:
        curentState = np.array([[0], [0], [0], [0]])
    if len(path) == 1:
        path = np.array([[0], [0], [0], [0]])

    assert len(curentState) > 3, "current state has the incorrect length"
    assert len(path) > 3, "current path has the incorrect length"

    vehicle = vehicleDynamicsJap(dt)

    # Saving data 
    TakingInputs = []
    NextState = []
    error = []
    desiredState = []


    # Get the next state
    # state_ = vehicle.nextX(state_ , input_)

    # Reshaping data
    # X = np.reshape(dummyDataX, (dummyDataX.shape[0], 1))
    # velX = np.zeros(X.shape)
    # Y = np.reshape(dummyDataY, (dummyDataY.shape[0], 1))
    # velY = np.zeros(Y.shape)
    # target = np.concatenate((X,velX,Y,velY), axis= 1)

    # Calculate control input
    print(f"we are in the mpc file this is the shape of currentPath; {curentState.reshape((4,)).shape}")
    print(f"we are in the mpc file this is the shape of path; { path[t,:].shape}")
    
    input_   = mpcControl(vehicle, 10, curentState.reshape((4,)), path[t, :])
    input_ = np.array(input_[0])
    input_ = np.reshape(input_, (input_.shape[0], 1))

    #Append to the lists
    # TakingInputs.append(input_)
    # NextState.append(state_)
    # error.append(state_ - target[i,:] )
    # desiredState.append(target[i,:])
    #

    # fig, axs = plt.subplots(2, figsize=(15, 15))
    # fig.suptitle('MPC implementation vehicle dynamics')
    # time = np.arange(0,len(np.array(NextState)[:,0]),1)
    # axs[0].plot(np.array(NextState)[:,0],np.array(NextState)[:,2] , label = "Position robot")
    # axs[0].plot(np.array(desiredState)[:,0],np.array(desiredState)[:,2] , label = "Desired Postion")
    #
    # axs[1].plot(np.array(NextState)[:,1],np.array(NextState)[:,3] , label = "Vel robot")
    # axs[1].plot(np.array(desiredState)[:,1],np.array(desiredState)[:,3] , label = "Desired Vel")
    # # axs[0].plot(target[:,0],target[:,1], label = "Desired position")
    # # axs[1].plot(np.arange(0, len(sumErrorOverColumns),1),sumErrorOverColumns, label = "Error" )
    #
    # axs[0].legend()
    # axs[1].legend()
    
    return input_
