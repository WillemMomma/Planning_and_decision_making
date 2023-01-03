import matplotlib.pyplot as plt
import numpy as np

def plot(ax,input, target):
   
    input = input[2:]

    for i in range(len(input)):
        ax.plot(input[:i,0],input[:i,1])
        ax.plot(target[:i,0],target[:i,1])
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