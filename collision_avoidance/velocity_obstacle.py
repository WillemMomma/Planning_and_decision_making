# # Import from libraries
# from matplotlib import pyplot as plt
# from shapely.geometry import Point
# import numpy as np
#
# # Import from other .py files
#
# from collision_avoidance.collision_check_functions import detect, collision_check, resolve
# from collision_avoidance.robot_class import Robot
#
# # Will update this branch to take into account vehicle dynamics
#
# def mainCollisionAvoidance(robot_list=None):
#
#
#     # If no input is given fill the robots list with the given data
#     if robot_list is None:
#         # Setting 1
#         robot1 = Robot(0, 0, 0.2, 0.5, 0, np.pi/2, True)
#         robot2 = Robot(0, 10, 0.2, 0, 0, 0, False)
#         robot_list = [robot1, robot2]
#
#     plotting = True
#
#     if plotting:
#         # Setup plot
#         plt.cla()
#         plt.minorticks_on()
#         plt.axis('equal')
#         plt.xlim(0, 5)
#         plt.ylim(-3, 3)
#
#     # Get the robot we're controlling from the whole list
#     our_robot = None
#     for robot in robot_list:
#         if robot.our:
#             our_robot = robot
#             break
#
#     # Set timestep
#     dt = our_robot.dt
#
#     # Detect all velocity obstacles here
#     cones = detect(robot_list, plotting)
#
#     # Check for collision with any of the obstacles for desired velocity
#     input_velocity = Point(our_robot.x + our_robot.input_vx, our_robot.y + our_robot.input_vy)
#     previous_velocity = Point(our_robot.x + our_robot.previous_vx, our_robot.y + our_robot.previous_vy)
#
#     # If no collision is found with desired velocity, continue with desired velocity
#     if not collision_check(input_velocity, cones):
#         our_robot.output_v = our_robot.input_v
#         our_robot.output_vx = our_robot.input_vx
#         our_robot.output_vy = our_robot.input_vy
#         our_robot.output_w = our_robot.input_w
#
#     # If collision is found with desired velocity, check for previous velocity
#     elif not collision_check(previous_velocity, cones):
#         our_robot.output_v = our_robot.previous_v
#         our_robot.output_vx = our_robot.previous_vx
#         our_robot.output_vy = our_robot.previous_vy
#         our_robot.output_w = our_robot.previous_w
#
#     # Else sample a new velocity
#     else:
#         resolve(our_robot, cones)
#
#     # Save current output for next timestep
#     our_robot.previous_v = our_robot.output_v
#     our_robot.previous_vx = our_robot.output_vx
#     our_robot.previous_vy = our_robot.output_vy
#     our_robot.previous_w = our_robot.output_w
#
#     if plotting:
#         # Draw and move the robots for this timestep
#         for robot in robot_list:
#             robot.draw(plt)
#
#         # Show and close plot
#         plt.show(block=False)
#         plt.pause(0.1)
#
#     return robot_list
#
# from model_predictive_control.uni_cycle_model import UniCycleModel
#
# uni = UniCycleModel(0.1)
#
# placeholderPos = np.zeros((2, 2))
# placeholderPos[1, :] = [0, 10]
# placeholderVel = np.zeros((2,))
# placeholderVel[0] = 1
# placeholderOr = np.zeros((2,))
# placeholderOr[0] = np.pi/2
# placeholderTra = np.zeros((100, 2))
# currentPositions, currentVelocities, currentOrientations, trajectory = [placeholderPos,
#                                                                         placeholderVel,
#                                                                         placeholderOr,
#                                                                         placeholderTra]
#
# robot_list = None
# angularVelocity = 0
# for i in range(200):
#     if i == 0:
#         robot_list = mainCollisionAvoidance()
#     else:
#         for i in range(len(robot_list)):
#             if i == 0:
#                 robot_list[i].update(currentPositions[i, 0],
#                                      currentPositions[i, 1],
#                                      currentVelocities[i],
#                                      angularVelocity,
#                                      currentOrientations[i])
#             else:
#                 robot_list[i].update(currentPositions[i, 0],
#                                      currentPositions[i, 1],
#                                      currentVelocities[i],
#                                      0,
#                                      currentOrientations[i])
#
#         robot_list = mainCollisionAvoidance(robot_list)
#         velocity = robot_list[0].output_v
#         currentVelocities[0] = robot_list[0].output_w
#
#         godert_input = np.array([currentVelocities[0], angularVelocity])
#         xytheta = uni.nextX(godert_input.reshape((1, 2)))
#         xy = xytheta[:2]
#         currentPositions[0, :] = xy.flatten()
#         currentOrientations[0] = xytheta[2]
