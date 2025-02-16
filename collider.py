import numpy as np
import matplotlib.pyplot as plt

# Define the start and end points
start_x, start_y = 0, 0
end_x, end_y = 10, 0
peak_y = 15

# Calculate the midpoint
mid_x = (start_x + end_x) / 2

# Calculate the coefficients for the parabola
a = (peak_y - start_y) / (mid_x - start_x)**2
b = 0  # Because the peak is at the midpoint, symmetry means b = 0
c = start_y

# Define the parabolic function
def parabolic_path(x):
    return a * (x - mid_x)**2 + c

# Generate x values
x_values = np.linspace(start_x, end_x, 400)

# Calculate corresponding y values
y_values = parabolic_path(x_values)

# Plot the path
plt.plot(x_values, y_values, label='Parabolic Path')
plt.scatter([start_x, mid_x, end_x], [start_y, peak_y, end_y], color='red', zorder=5)
plt.text(start_x, start_y, 'Start', fontsize=12, verticalalignment='bottom', horizontalalignment='right')
plt.text(mid_x, peak_y, 'Peak', fontsize=12, verticalalignment='bottom', horizontalalignment='right')
plt.text(end_x, end_y, 'End', fontsize=12, verticalalignment='bottom', horizontalalignment='right')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Parabolic Path')
plt.legend()
plt.grid(True)
plt.show()
