import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as colors 
import matplotlib.patches as patches

plot_multi_range_single_speed = False; at_which_speed = 240 # km/h
plot_multi_range_optimal_speed = False
plot_multi_range_multi_speed_optimal_config = False
plot_multi_range_multi_speed_optimal_config_around_uber_vehicle_requirement = False

# Mission requirement sweep
range_i, range_f, d_range = 10, 230, 10 	# km
speed_i, speed_f, d_speed = 80, 330, 10 	# km/h
range_array = np.arange(range_i,range_f,d_range)
speed_array = np.arange(speed_i,speed_f,d_speed)

range_grid, speed_grid = np.meshgrid(range_array, speed_array)

if plot_multi_range_optimal_speed:

	data_df = pd.read_csv('multirotor/Multirotor_OPT_results_with_speed_as_design_var.csv')
	data_df2 = pd.read_csv('liftcruise/Liftcruise_OPT_results_with_speed_as_design_var.csv')

	data_df = data_df[data_df['Weight|residual']<0.1]
	data_df = data_df[data_df['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|Cruise|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|Cruise|mu']<1.1]
	data_df = data_df[data_df['LiftRotor|Cruise|CT/sigma']<0.15]

	data_df2 = data_df2[data_df2['Weight|residual']<0.1]
	data_df2 = data_df2[data_df2['Aero|Cruise|CL']<0.91]
	data_df2 = data_df2[data_df2['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|J']<3.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|CT/sigma']<0.15]
	data_df2 = data_df2[data_df2['LiftRotor|clearance_constraint']<0.1]

	plt.plot(data_df['mission_range'], data_df['Weight|takeoff'], '-o', label=f'Multirotor')
	plt.plot(data_df2['mission_range'], data_df2['Weight|takeoff'], '-o', label=f'Lift+cruise')
	plt.xlabel('Mission range [km]')
	plt.ylabel('Optimal MTOW [kg]')
	plt.legend(loc='upper left')
	plt.title(f'Optimal MTOW at optimal speeds')
	plt.grid()
	plt.show()

if plot_multi_range_single_speed:

	data_df = pd.read_csv('multirotor/Multirotor_OPT_results_without_speed_as_design_var.csv')
	data_df2 = pd.read_csv('liftcruise/Liftcruise_OPT_results_without_speed_as_design_var.csv')

	data_df = data_df[data_df['Weight|residual']<0.1]
	data_df = data_df[data_df['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|Cruise|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|Cruise|mu']<1.1]
	data_df = data_df[data_df['LiftRotor|Cruise|CT/sigma']<0.15]

	data_df2 = data_df2[data_df2['Weight|residual']<0.1]
	data_df2 = data_df2[data_df2['Aero|Cruise|CL']<0.91]
	data_df2 = data_df2[data_df2['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|J']<3.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|CT/sigma']<0.15]
	data_df2 = data_df2[data_df2['LiftRotor|clearance_constraint']<0.1]

	data = data_df[data_df['cruise_speed']==at_which_speed]
	data2 = data_df2[data_df2['cruise_speed']==at_which_speed]

	plt.plot(data['mission_range'], data['Weight|takeoff'], '-o', label=f'Multirotor')
	plt.plot(data2['mission_range'], data2['Weight|takeoff'], '-o', label=f'Lift+cruise')
	plt.xlabel('Mission range [km]')
	plt.ylabel('Optimal MTOW [kg]')
	plt.legend(loc='upper left')
	plt.title(f'Optimal MTOW at cruise speed= {at_which_speed} km/h')
	plt.grid()
	plt.show()

if plot_multi_range_multi_speed_optimal_config or plot_multi_range_multi_speed_optimal_config_around_uber_vehicle_requirement:

	data_df = pd.read_csv('multirotor/Multirotor_OPT_results_without_speed_as_design_var.csv')
	data_df2 = pd.read_csv('liftcruise/Liftcruise_OPT_results_without_speed_as_design_var.csv')

	# Filtering
	# data_df = data_df[data_df['Weight|residual']<1.0]
	# data_df2 = data_df2[data_df2['Weight|residual']<1.0]
	data_df.loc[data_df['Weight|residual'] > 0.1, 'Weight|takeoff'] = np.nan
	data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
	data_df.loc[data_df['LiftRotor|Cruise|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
	data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
	data_df.loc[data_df['LiftRotor|Cruise|mu'] > 1, 'Weight|takeoff'] = np.nan
	data_df.loc[data_df['LiftRotor|Cruise|CT/sigma'] > 0.15, 'Weight|takeoff'] = np.nan

	data_df.loc[data_df['Weight|residual'] > 0.1, 'Energy|entire_mission'] = np.nan
	data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
	data_df.loc[data_df['LiftRotor|Cruise|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
	data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
	data_df.loc[data_df['LiftRotor|Cruise|mu'] > 1, 'Energy|entire_mission'] = np.nan
	data_df.loc[data_df['LiftRotor|Cruise|CT/sigma'] > 0.15, 'Energy|entire_mission'] = np.nan

	data_df2.loc[data_df2['Weight|residual'] > 0.1, 'Weight|takeoff'] = np.nan
	data_df2.loc[data_df2['Aero|Cruise|CL'] > 0.91, 'Weight|takeoff'] = np.nan
	data_df2.loc[data_df2['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
	data_df2.loc[data_df2['Propeller|Cruise|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
	data_df2.loc[data_df2['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
	data_df2.loc[data_df2['Propeller|Cruise|CT/sigma'] > 0.15, 'Weight|takeoff'] = np.nan
	data_df2.loc[data_df2['Propeller|Cruise|J'] > 3.1, 'Weight|takeoff'] = np.nan

	data_df2.loc[data_df2['Weight|residual'] > 0.1, 'Energy|entire_mission'] = np.nan
	data_df2.loc[data_df2['Aero|Cruise|CL'] > 0.91, 'Energy|entire_mission'] = np.nan
	data_df2.loc[data_df2['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
	data_df2.loc[data_df2['Propeller|Cruise|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
	data_df2.loc[data_df2['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
	data_df2.loc[data_df2['Propeller|Cruise|CT/sigma'] > 0.15, 'Energy|entire_mission'] = np.nan
	data_df2.loc[data_df2['Propeller|Cruise|J'] > 3.1, 'Energy|entire_mission'] = np.nan

	# Generate data
	MTOW = data_df['Weight|takeoff'].to_numpy().reshape(len(range_array), len(speed_array)).T
	MTOW2 = data_df2['Weight|takeoff'].to_numpy().reshape(len(range_array), len(speed_array)).T
	Energy = data_df['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T
	Energy2 = data_df2['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T
	data = np.empty((len(speed_array), len(range_array)))
	data.fill(np.nan)
	data2 = np.empty((len(speed_array), len(range_array)))
	data2.fill(np.nan)

	for i in range(len(speed_array)):
		for j in range(len(range_array)):
			if not np.isnan(MTOW[i][j]) and not np.isnan(MTOW2[i][j]):
				if MTOW[i][j] < MTOW2[i][j]:
					data[i][j] = 0.0
				else:
					data[i][j] = 1.0
			elif np.isnan(MTOW[i][j]) and not np.isnan(MTOW2[i][j]):
				data[i][j] = 1.0
			elif not np.isnan(MTOW[i][j]) and np.isnan(MTOW2[i][j]):
				data[i][j] = 0.0
			else:
				data[i][j] = 2.0
	
	for i in range(len(speed_array)):
		for j in range(len(range_array)):
			if not np.isnan(Energy[i][j]) and not np.isnan(Energy2[i][j]):
				if Energy[i][j] < Energy2[i][j]:
					data2[i][j] = 0.0
				else:
					data2[i][j] = 1.0
			elif np.isnan(Energy[i][j]) and not np.isnan(Energy2[i][j]):
				data2[i][j] = 1.0
			elif not np.isnan(Energy[i][j]) and np.isnan(Energy2[i][j]):
				data2[i][j] = 0.0
			else:
				data2[i][j] = 2.0

	# print(data)
	# print(data2)

	# Create a custom color map
	colors_list = ['#404788ff', '#55c667ff', '#fde725ff']
	cmap = colors.ListedColormap(colors_list)

	if plot_multi_range_multi_speed_optimal_config:

		fig, ax = plt.subplots()

		# Plot heatmap of optimal config by MTOW
		im = ax.imshow(data, cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
				   	   extent=[range_array[0]-d_range/2, range_array[-1]+d_range/2,
				   	   speed_array[0]-d_speed/2, speed_array[-1]+d_speed/2])

		# Annotate area of interest of Uber Vehicle Requirement
		rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='Uber Vehicle Requirement')
		ax.add_patch(rect)

		cbar = fig.colorbar(im, ticks=[0.33, 1, 1.66])
		cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])
		ax.set_title('Optimal configuration by MTOW')
		ax.set_xlabel('Mission range [km]')
		ax.set_ylabel('Cruise speed [km/h]')
		ax.legend()
		plt.show()

		fig, ax = plt.subplots()

		# Plot heatmap of optimal config by Energy
		im = ax.imshow(data2, cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
					   extent=[range_array[0]-d_range/2, range_array[-1]+d_range/2,
				   	   speed_array[0]-d_speed/2, speed_array[-1]+d_speed/2])

		# Annotate area of interest of Uber Vehicle Requirement
		rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='Uber Vehicle Requirement')
		ax.add_patch(rect)

		cbar = fig.colorbar(im, ticks=[0.33, 1, 1.66])
		cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])
		ax.set_title('Optimal configuration by required energy')
		ax.set_xlabel('Mission range [km]')
		ax.set_ylabel('Cruise speed [km/h]')
		ax.legend()
		plt.show()

	if plot_multi_range_multi_speed_optimal_config_around_uber_vehicle_requirement:

		fig, ax = plt.subplots()

		# Plot heatmap of optimal config by MTOW
		im = ax.imshow(data, cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
				  	   extent=[range_array[0]-d_range/2, range_array[-1]+d_range/2,
				   	   speed_array[0]-d_speed/2, speed_array[-1]+d_speed/2])

		# Annotate area of interest of Uber Vehicle Requirement
		rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='Uber Vehicle Requirement')
		ax.add_patch(rect)

		cbar = fig.colorbar(im, ticks=[0.33, 1, 1.66])
		cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])
		ax.set_title('Optimal configuration by MTOW')
		ax.set_xlabel('Mission range [km]')
		ax.set_ylabel('Cruise speed [km/h]')
		ax.set_ylim([160, 320])
		ax.legend()
		plt.show()

		fig, ax = plt.subplots()

		# Plot heatmap of optimal config by MTOW
		im = ax.imshow(data2, cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
				  	   extent=[range_array[0]-d_range/2, range_array[-1]+d_range/2,
				   	   speed_array[0]-d_speed/2, speed_array[-1]+d_speed/2])

		# Annotate area of interest of Uber Vehicle Requirement
		rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='Uber Vehicle Requirement')
		ax.add_patch(rect)

		cbar = fig.colorbar(im, ticks=[0.33, 1, 1.66])
		cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])
		ax.set_title('Optimal configuration by required energy')
		ax.set_xlabel('Mission range [km]')
		ax.set_ylabel('Cruise speed [km/h]')
		ax.set_ylim([160, 320])
		ax.legend()
		plt.show()

