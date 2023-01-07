# Global planner 
This folder contains two global planning algorithms, RRT and RRT*. 
The results presented are obtained using RRT*, however if one cares less about optimality of the path found 
and more about time efficiency of the computations, RRT could be used since it is much faster. 

## RRT*
To test RRT* on its own simply run the main function:

    Main function
    Specify start, goal, obstacles, animation (set False for time results) and random area
    
    optional settings:
        maxIter: maximum number of iterations, default 1200
        maxExpansion: maximum distance to expand tree, default 3
        r: radius of circle around node to check for collision, default y(log n/n)^(1/3)
        probGoal: probability of selecting goal as random node, default 0.05
        threshold: threshold to check if goal is reached, default 0.5
        searchGamma: gamma value for RRT*, default 40
    
    output: path from start to goal, interpolated path

The visualisation is quite slow, set animation to False for optimal time solution. 
Increase number of iterations for more optimal solution. 
Decrease maxExpansion for smaller in between steps
Increase Gamma to look in a larger radius for more optimal solution
