import matplotlib.pyplot as plt
from shapely.geometry import LineString
import shapely

def lineCollisionCheck(obstacleList, x1, y1, x2, y2):
    '''
    Check if line between two nodes collides with any obstacle
    Input:  node1: start node
            node2: end node
            obstacleList: list of obstacles as polygons [obstacle1, obstacle2, ...]
    Output: True if no collision, False if collision
    '''
    #create line object 
    line = LineString([(x1,y1),(x2,y2)])
    for obs in obstacleList:
        r=0
        polygon = shapely.geometry.box(obs[0]-r, obs[1]-r, obs[2]+r, obs[3]+r)
        if line.intersects(polygon): #check if line intersects with polygon
            return False
    return True



def visualize_line_intersection(rectangles, x1, y1, x2, y2):
    # Plot the rectangles
    for r_x1, r_y1, r_x2, r_y2 in rectangles:
        plt.plot([r_x1, r_x2, r_x2, r_x1, r_x1], [r_y1, r_y1, r_y2, r_y2, r_y1], 'b')

    # Plot the line
    plt.plot([x1, x2], [y1, y2], 'r')

    # Set the plot limits
    plt.xlim(min(x1, x2, *[r_x1 for r_x1, _, _, _ in rectangles]), max(x1, x2, *[r_x2 for _, _, r_x2, _ in rectangles]))
    plt.ylim(min(y1, y2, *[r_y1 for _, r_y1, _, _ in rectangles]), max(y1, y2, *[r_y2 for _, _, _, r_y2 in rectangles]))

    # Show the plot
    plt.show()

rectangles = [
    (1, 1, 5, 5),  
    (10, 10, 20, 20),
    (15, 15, 25, 25),
    (30, 30, 40, 40),
    (35, 35, 45, 45),
    (20,30,21,43)

]
line1 = (6, 25, 13, 30)
line2 = (30, 15, 40, 25)
line3 = (1, 1, 5, 5)
line4 = (4,4,10,10)
line5 = (25,25,30,30)
line6 = (6,18,13,23)
line7 = (19,32,24,37)
lines = [line1, line2, line3, line4, line5, line6, line7]

for line in lines:
    visualize_line_intersection(rectangles, line[0], line[1], line[2], line[3])
    print(lineCollisionCheck(rectangles, line[0], line[1], line[2], line[3])) 
