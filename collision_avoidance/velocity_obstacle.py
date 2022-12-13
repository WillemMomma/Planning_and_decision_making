# Import from libraries
from matplotlib import pyplot as plt
from shapely.geometry import Point

# Import from other .py files
from robot_class import Robot
from collision_check_functions import detect, collision_check, resolve

# Setting 1
robot1 = Robot(0, 0, 1, 0, 0.5, True)
robot2 = Robot(-10, 10, 1, 0.5, 0, False)
robot3 = Robot(3, 5, 1, 0, 0, False)
robot4 = Robot(10, 15, 1, -0.5, 0, False)
robot_list = [robot1, robot2, robot3, robot4]

# # Setting 2
# robot1 = Robot(0, 0, 1, 0, 0.5, True)
# robot2 = Robot(0, 15, 1, 0, 0, False)
# robot3 = Robot(10, 17, 1, -0.5, 0, False)
# robot4 = Robot(-10, 13, 1, 0.5, 0, False)
# robot_list = [robot1, robot2, robot3, robot4]


# Setup plot
fig = plt.figure()
plt.minorticks_on()
plt.axis('equal')
plt.xlim(-15, 15)
plt.ylim(-15, 15)

# Get the robot we're controlling from the whole list
our_robot = None
for robot in robot_list:
    if robot.our:
        our_robot = robot

# Main loop
T = 40
dt = 0.25
N = int(T/dt)
for t in range(N):

    # Clear plot from last iteration
    plt.cla()

    # Detect all velocity obstacles here
    cones = detect(robot_list)

    # Check for collision with any of the obstacles for desired velocity
    desired_velocity = Point(our_robot.x + our_robot.desired_vx, our_robot.y + our_robot.desired_vy)
    previous_velocity = Point(our_robot.x + our_robot.previous_vx, our_robot.y + our_robot.previous_vy)

    # If no collision is found with desired velocity, continue with desired velocity
    if not collision_check(desired_velocity, cones):
        our_robot.output_vx = our_robot.desired_vx
        our_robot.output_vy = our_robot.desired_vy

    # If collision is found with desired velocity, check for previous velocity
    elif not collision_check(previous_velocity, cones):
        our_robot.output_vx = our_robot.previous_vx
        our_robot.output_vy = our_robot.previous_vy

    # Else sample a new velocity
    else:
        resolve(our_robot, cones)

    # Save current output for next timestep
    our_robot.previous_vx = our_robot.output_vx
    our_robot.previous_vy = our_robot.output_vy

    # Draw and move the robots for this timestep
    for robot in robot_list:
        robot.draw(plt)
        robot.move(dt)

    plt.plot(block='False')
    plt.pause(0.001)

plt.show()
