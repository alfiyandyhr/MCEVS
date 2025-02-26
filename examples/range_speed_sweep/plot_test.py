import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

plot_2D = True

if plot_2D:
	# data_df = pd.read_csv('multirotor/Multirotor_OPT_results_without_speed_as_design_var.csv')
	data_df = pd.read_csv('multirotor/Multirotor_OPT_results_test.csv')
	# data_df2 = pd.read_csv('liftcruise/Liftcruise_OPT_results_without_speed_as_design_var.csv')
	data_df2 = pd.read_csv('liftcruise/Liftcruise_OPT_results_test.csv')	
	data_df = data_df[data_df['Weight|residual']<1.0]
	data_df2 = data_df2[data_df2['Weight|residual']<1.0]

	range_array = np.unique(data_df['mission_range'].to_numpy())
	speed_array = np.unique(data_df['cruise_speed'].to_numpy())

	for cruise_speed in speed_array:
		data = data_df[data_df['cruise_speed']==cruise_speed]
		data2 = data_df2[data_df2['cruise_speed']==cruise_speed]

		# if cruise_speed in [36, 72, 108, 144, 180, 216]:
		# if cruise_speed in [216, 252, 288, 324, 360, 396, 432, 468, 504, 540, 576]:
		if cruise_speed == 252:
			plt.plot(data['mission_range'], data['Weight|takeoff'], '-ro', label=f'Cruise speed = {cruise_speed}')
			plt.plot(data2['mission_range'], data2['Weight|takeoff'], '-bo', label=f'Cruise speed = {cruise_speed}')

	# plt.plot(data2['mission_range'], data2['Weight|takeoff'], 'k-', label=f'Cruise speed = optimal')
	plt.xlabel('Mission range [km]')
	plt.ylabel('Optimal MTOW [kg]')
	plt.legend()
	plt.grid()
	plt.show()

	# for cruise_speed in speed_array:
	# 	data = data_df[data_df['cruise_speed']==cruise_speed]

	# 	if cruise_speed in [216, 252, 288, 324, 360, 396, 432, 468, 504, 540, 576]:
	# 		plt.plot(data['mission_range'], data['Weight|takeoff'], '-o', label=f'Cruise speed = {cruise_speed}')

	# # plt.plot(data2['mission_range'], data2['Weight|takeoff'], 'k-', label=f'Cruise speed = optimal')
	# plt.xlabel('Mission range [km]')
	# plt.ylabel('Optimal MTOW [kg]')
	# plt.legend()
	# plt.grid()
	# plt.show()