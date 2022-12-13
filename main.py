# Importing libraries 
import env from env.env
import MPC from model_predictive_control.model_predictive_control
import global_planning from global_planning.global_planning as globalPlanning
import collision_avoidance from collision_avoidance.collision_avoidance as collisionAvoidance


def behaviour():
    """
    Start the enviroment and run the algorithms

    input -> None : None
    Ouput -> None : None
    """
    # Initialize enviroment here
    map = 0    #calling the map function inside the env
    map = env(map)
    states = env(states)

    # Genrate the global trajectory here
    trajectory = globalPlanning(states)

    # Start in the correct state
    state = 0
    run = True 
    
    while run: 
        timestep = 0

        if state == 0: # running
            collisionFreePath = collisionAvoidance(trajectory, states, timestep)
            action = MPC(collisionFreePath["trajectory"], collisionFreePath["velocites"])
            states = env(action)

            # Check if the state has chaged
            state  = collisionFreePath["status"]   

            """"
            collisionFreePath = {trajectory : np.array([]),
                                 velocities : np.array([])
                                 status     : Int,
                                 done       : Int}                  
            """
            
        if state == 1: # Calculate new trajectory
            trajectory = globalPlanning(states)
            state = trajectory["status"]

        if state == 2: # No possible solution
            print("No path can be found")
            run = False

        if state == 3: # No possible solution
            print("Finished")
            run = False

        timestep += 1 

    # Closing the enviroment 
    env(close)
    return 
        

if __name__ == "__main__":
    behaviour()