import matplotlib.pyplot as plt

def line_intersects_rectangle(x1, y1, x2, y2, r_x1, r_y1, r_x2, r_y2):
    # Check if either of the line's endpoints are inside the rectangle
    inside1 = r_x1 <= x1 <= r_x2 and r_y1 <= y1 <= r_y2
    inside2 = r_x1 <= x2 <= r_x2 and r_y1 <= y2 <= r_y2
    if inside1 or inside2:
        return True

    # Check if the line intersects any of the rectangle's sides
    for r_x, r_y in [(r_x1, r_y1), (r_x2, r_y1), (r_x2, r_y2), (r_x1, r_y2)]:
        if line_intersects_point(x1, y1, x2, y2, r_x, r_y):
            return True

    return False

def line_intersects_point(x1, y1, x2, y2, px, py):
    # Check if the point is strictly inside the box formed by the line's endpoints
    if (x1 < px < x2 or x2 < px < x1) and (y1 < py < y2 or y2 < py < y1):
        return True

    # Check if the point is on the line
    if (px == x1 and py == y1) or (px == x2 and py == y2):
        return True

    # Check if the point is collinear with the line and lies on the line segment
    if (x1 == x2 and x1 == px) or (y1 == y2 and y1 == py):
        if (y1 <= py <= y2 or y2 <= py <= y1) and (x1 <= px <= x2 or x2 <= px <= x1):
            return True

    return False


def line_intersects_any_rectangle(rectangles, x1, y1, x2, y2):
    for r_x1, r_y1, r_x2, r_y2 in rectangles:
        if line_intersects_rectangle(x1, y1, x2, y2, r_x1, r_y1, r_x2, r_y2):
            return True
    return False



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
    (35, 35, 45, 45)

]
line1 = (6, 25, 13, 30)
line2 = (30, 15, 40, 25)
line3 = (1, 1, 5, 5)
line4 = (4,4,10,10)
line5 = (10,1,2,10)
lines = [line1, line2, line3, line4, line5]

for line in lines:
    visualize_line_intersection(rectangles, line[0], line[1], line[2], line[3])
    print(line_intersects_any_rectangle(rectangles, line[0], line[1], line[2], line[3])) 
