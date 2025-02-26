import numpy as np
import matplotlib.pyplot as plt

# Create a grid
grid_size = (10, 10)
grid = np.zeros(grid_size)

# Define obstacles
obstacles = [(3, 3), (3, 4), (4, 4), (5, 5)]
for obs in obstacles:
    grid[obs] = 1  # Mark obstacles

# Define start & goal
start = (0, 0)
goal = (9, 9)

# Example path
path = [(0,0), (1,1), (2,2), (3,2), (4,3), (5,4), (6,5), (7,6), (8,7), (9,9)]

# Plot the grid
plt.figure(figsize=(6,6))
plt.imshow(grid, cmap='gray_r', origin='lower')

# Plot obstacles
for obs in obstacles:
    plt.scatter(obs[1], obs[0], marker='s', color='black', s=200)

# Plot start, goal, and path
plt.scatter(start[1], start[0], color='green', s=100, label="Start")
plt.scatter(goal[1], goal[0], color='red', s=100, label="Goal")

# Draw the path
path_x, path_y = zip(*path)
plt.plot(path_y, path_x, marker='o', color='blue', label="Path")

plt.legend()
plt.grid(True)
plt.title("Path Planning Visualization")
plt.show()

from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Example path in 3D
path_x = [0, 1, 2, 3, 4, 5]
path_y = [0, 1, 1, 2, 3, 4]
path_z = [0, 1, 2, 2, 3, 3]

ax.plot(path_x, path_y, path_z, marker='o', color='blue')

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.set_title("3D Path Planning")

plt.show()
