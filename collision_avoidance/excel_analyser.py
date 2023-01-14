import openpyxl
import matplotlib.pyplot as plt
import numpy as np

Test_cases = [2, 3]
Test_labels = ["Stationary", "Moving", "Moving and turning"]
results_GVO = {"stationary": {"velocity": [2.48, 0.29],
                              "distance": [1.17, 0.47]}}  # average, standard deviation
results_VO = {"stationary": {"velocity": [0.18, 0.10],
                             "distance": [4.08, 0.47]}}
results_VO_no_collision = {"stationary": {"velocity": [0.35, 0.43],
                                          "distance": [3.84, 0.89]}}

# I want to make a bar chart which gives for multiple test labels the average and standard deviation of the velocity and distance for all results dictionaries
# Plot the barchart in Python
def bar_chart(results, test_labels):
    # Create a figure
    fig, ax = plt.subplots()
    # Set the title
    ax.set_title("Average velocity and distance")
    # Set the x-axis label
    ax.set_xlabel("Test label")
    # Set the y-axis label
    ax.set_ylabel("Average velocity and distance")
    # Set the x-axis ticks
    ax.set_xticks([0, 1, 2])
    # Set the x-axis tick labels
    ax.set_xticklabels(test_labels)
    # Set the y-axis ticks
    ax.set_yticks([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
    # Set the y-axis tick labels
    ax.set_yticklabels(["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"])
    # Set the legend
    ax.legend(["Average velocity", "Average distance"])
    # Set the grid
    ax.grid()

    # Set the width of the bars
    width = 0.35
    # Set the x locations for the groups
    x = np.arange(len(test_labels))
    # Set the x locations for the bars
    x1 = [i - width / 2 for i in x]
    x2 = [i + width / 2 for i in x]

    # Plot the bars
    ax.bar(x1, [results[i]["velocity"][0] for i in results.keys()], width, yerr=[results[i]["velocity"][1] for i in results.keys()], label="Average velocity")
    ax.bar(x2, [results[i]["distance"][0] for i in results.keys()], width, yerr=[results[i]["distance"][1] for i in results.keys()], label="Average distance")

    # Show the plot
    plt.show()

bar_chart(results_GVO, Test_labels)
