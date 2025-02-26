import matplotlib.colors as colors 
import matplotlib.pyplot as plt
import numpy as np

range_i, range_f, d_range = 20, 60, 5
speed_i, speed_f, d_speed = 30, 70, 5

range_array = np.arange(range_i,range_f,d_range)
speed_array = np.arange(speed_i,speed_f,d_speed)

range_grid, speed_grid = np.meshgrid(range_array, speed_array)

# Generate random data
data = np.random.randint(0, 100, size=(range_array.size, speed_array.size))

# Create a custom color map with blue and green colors
colors_list = ['#0099ff', '#33cc33']
cmap = colors.ListedColormap(colors_list)

# Plot the heatmap with custom colors and annotations
plt.imshow(data, cmap=cmap, vmin=0, vmax=100,
		   extent=[range_array[0]-d_range/2, range_array[-1]+d_range/2,
		   		   speed_array[0]-d_speed/2, speed_array[-1]+d_speed/2])
# for i in range(range_array.size):
# 	for j in range(speed_array.size):
# 		plt.annotate(str(data[i][j]), xy=(range_array[i], speed_array[j]), ha='center', va='center', color='white')

# Add colorbar
cbar = plt.colorbar(ticks=[25, 75])
cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise'])

# Set plot title and axis labels 
plt.title('Range-speed sweep')
plt.xlabel('Range [km]')
plt.ylabel('Cruise speed [m/s]')

# Display the plot 
plt.show()