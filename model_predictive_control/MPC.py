import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm

# Inintialize variables
dt = 0.1
start = 0 
stop = 10 
dummyDataX = np.arange(start ,stop ,dt)
dummyDataY = np.ones(len(dummyDataX))
dummyDataY = np.sin(dummyDataX)

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
    
    constraints += [x[:, 0] == xInit]
        
    for k in range(N):
        # State space
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


def dummyRun():
    """
    Start simulating the run of the model and plot the results 
    
    input ->  None
    output-> Plots
    """

    lookahead = 10
    state_ = np.array([1,0,0,0])
    input_ = np.array([[0],[0],[0],[0]])
    vehicle = vehicleDynamicsJap(dt)
    target = np.zeros((4,4)) 

    # Saving data 
    TakingInputs = []
    NextState = []
    error = []
    desiredState = []

    for i in tqdm(range(len(dummyDataX))):

        # Get the next state
        state_ = vehicle.nextX(state_ , input_)

        # Reshaping data
        X = np.reshape(dummyDataX, (dummyDataX.shape[0], 1))
        velX = np.zeros(X.shape)
        Y = np.reshape(dummyDataY, (dummyDataY.shape[0], 1))
        velY = np.zeros(Y.shape)
        target = np.concatenate((X,velX,Y,velY), axis= 1)

        # Calculate control input
        input_   = mpcControl(vehicle, 10, state_, target[i,:])
        input_ = np.array(input_[0])
        input_ = np.reshape(input_, (input_.shape[0], 1))

        #Append to the lists
        TakingInputs.append(input_)
        NextState.append(state_)
        error.append(state_ - target[i,:] )
        desiredState.append(target[i,:])


    fig, axs = plt.subplots(2, figsize=(15, 15))
    fig.suptitle('MPC implementation vehicle dynamics')
    time = np.arange(0,len(np.array(NextState)[:,0]),1)
    axs[0].plot(np.array(NextState)[:,0],np.array(NextState)[:,2] , label = "Position robot")
    axs[0].plot(np.array(desiredState)[:,0],np.array(desiredState)[:,2] , label = "Desired Postion")

    axs[1].plot(np.array(NextState)[:,1],np.array(NextState)[:,3] , label = "Vel robot")
    axs[1].plot(np.array(desiredState)[:,1],np.array(desiredState)[:,3] , label = "Desired Vel")
    # axs[0].plot(target[:,0],target[:,1], label = "Desired position")
    # axs[1].plot(np.arange(0, len(sumErrorOverColumns),1),sumErrorOverColumns, label = "Error" )

    axs[0].legend()
    axs[1].legend()
    
    return
