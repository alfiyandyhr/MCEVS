import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as colors 

# Mission requirement sweep
range_i, range_f, d_range = 10, 160, 10 	# km
speed_i, speed_f, d_speed = 36, 600, 36 	# km/h
range_array = np.arange(range_i,range_f,d_range)
speed_array = np.arange(speed_i,speed_f,d_speed)

range_grid, speed_grid = np.meshgrid(range_array, speed_array)

# Data
data_df = pd.read_csv('multirotor/Multirotor_OPT_results_without_speed_as_design_var.csv')
data_df2 = pd.read_csv('liftcruise/Liftcruise_OPT_results_without_speed_as_design_var.csv')

# Filtering
# data_df = data_df[data_df['Weight|residual']<1.0]
# data_df2 = data_df2[data_df2['Weight|residual']<1.0]
data_df.loc[data_df['Weight|residual'] > 1, 'Weight|takeoff'] = 5000.0
data_df2.loc[data_df2['Weight|residual'] > 1, 'Weight|takeoff'] = 10000.0
data_df.loc[data_df['Weight|residual'] > 1, 'Energy|entire_mission'] = 5000.0
data_df2.loc[data_df2['Weight|residual'] > 1, 'Energy|entire_mission'] = 10000.0

# Generate data
MTOW = data_df['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T
MTOW2 = data_df2['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T
data = np.empty((len(speed_array), len(range_array)))
data.fill(np.nan)

i=12; j=12
print(MTOW[i][j], MTOW2[i][j])

# for i in range(len(speed_array)):
# 	for j in range(len(range_array)):
# 		if MTOW[i][j] != 5000.0 and MTOW2[i][j] != 10000.0:
# 			if MTOW[i][j] < MTOW2[i][j]:
# 				data[i][j] = 0.0
# 			else:
# 				data[i][j] = 1.0
# print(data)

# print(MTOW-MTOW2)
# # Generate random data
# data = np.random.randint(0, 100, size=(range_array.size, speed_array.size))

# # Create a custom color map with blue and green colors
# colors_list = ['#0099ff', '#33cc33']
# cmap = colors.ListedColormap(colors_list)

# # Plot the heatmap with custom colors and annotations
# plt.imshow(data, cmap=cmap, vmin=0, vmax=100, aspect='auto',
# 		   extent=[range_array[0]-d_range/2, range_array[-1]+d_range/2,
# 		   		   speed_array[0]-d_speed/2, speed_array[-1]+d_speed/2])
# # for i in range(range_array.size):
# # 	for j in range(speed_array.size):
# # 		plt.annotate(str(data[i][j]), xy=(range_array[i], speed_array[j]), ha='center', va='center', color='white')

# # Add colorbar
# cbar = plt.colorbar(ticks=[25, 75])
# cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise'])

# # Set plot title and axis labels 
# plt.title('Range-speed sweep')
# plt.xlabel('Range [km]')
# plt.ylabel('Cruise speed [km/h]')

# # Display the plot 
# plt.show()