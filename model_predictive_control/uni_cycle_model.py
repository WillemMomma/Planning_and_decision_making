import numpy as np

class UniCycleModel:
    """
    Defining the vehicle dynamics as an LTV this model is used to test the behaviour of the unicycle
    """

    def __init__(self, dt):
        # Euler discretization
        self.A = np.eye(3)
        self.X = np.array([[1],[0],[-np.pi/2]])
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