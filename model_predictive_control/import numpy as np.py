import numpy as np 

dummyDataX = np.arange(0 ,10 ,0.1)
dummyDataY = np.arange(0 ,1 ,0.01)
print(np.vstack((dummyDataX, dummyDataY)).shape)
