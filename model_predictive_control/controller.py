import numpy as np
import cvxpy as cp


def mpcControl(error, N, xInit, xTarget):
    """
    Control the vehicle using MPC it is assumed that the operater can only alter the acceleration
    
    Normally we would have an LTI, but now we have an LTV therefore
    x[t+1] = A*x + B*u
    
    The state of the bot   ->  [x, y, theta] 
    X represents the error -> [xError, yError, thereError]
    U represents the input -> [v,theta]

    A end B are error matrices packaged in the error variable

    """
    
    weightInput = np.array([[1,0],[0,0.01]])    # Weight on the input
    weightTracking = np.array([[10,0,0],[0,10,0],[0,0,10]]) # Weight on the tracking state
    
    cost = 0.
    constraints = []
    
    # Create the optimization variables
    x = cp.Variable((3, N + 1)) # cp.Variable((dim_1, dim_2))
    u = cp.Variable((2, N))


    # Initialize the current error
    constraints += [x[:, 0] == error[1].reshape((3,))]  
        
    for k in range(N):
        
        nextError = error[0].reshape((3,3))@x[:,k] + error[2].reshape((3,2))@(u[:, k])

        # constraints
        constraints += [x[:, k+1] == nextError]
        constraints += [u[:, k] <= [6,3]]
        constraints += [u[:, k] >= [0,-3]]

        # Minimize the cost function
        cost += cp.quad_form(u[:, k], weightInput)
        cost += cp.quad_form((x[:, k+1] ), weightTracking)
    
    # Terminal set
    cost += cp.quad_form(x[:, N], weightTracking*10000)

    
    # Solves the problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.OSQP)

    # We return the MPC input and the next state (and also the plan for visualization)
    return u[:, 0].value - error[3] , x[:, 1].value, x[:, :].value, None


def PID():
    pass