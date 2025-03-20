import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

plot_minimizing_takeoff_weight = False
plot_minimizing_energy = False
plot_minimizing_mission_time = False

plot_minimizing_all_objs = True

configuration = 'multirotor'
# configuration = 'liftcruise'

if plot_minimizing_takeoff_weight or plot_minimizing_energy or plot_minimizing_mission_time:

	if plot_minimizing_takeoff_weight:
		folder = 'minimizing_takeoff_weight'
		label = 'Weight-minimized'
		df_label = 'Weight|takeoff'
		y_label = 'Takeoff weight [kg]'
	if plot_minimizing_energy:
		folder = 'minimizing_energy'
		label = 'Energy-minimized'
		df_label = 'Energy|entire_mission'
		y_label = 'Energy [kWh]'
	if plot_minimizing_mission_time:
		folder = 'minimizing_mission_time'
		label = 'Time-minimized'
		df_label = 'mission_time'
		y_label = 'Mission time [mins]'

	data_df1 = pd.read_csv(f'{folder}/{configuration}/battery_250_Whpkg/results_with_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'{folder}/{configuration}/battery_400_Whpkg/results_with_speed_as_design_var.csv')
	data_df3 = pd.read_csv(f'{folder}/{configuration}/battery_550_Whpkg/results_with_speed_as_design_var.csv')
	data_list = [data_df1, data_df2, data_df3]
	
	if configuration == 'multirotor':
		for i in range(3):
			data_list[i] = data_list[i][data_list[i]['Weight|residual']<0.1]
			data_list[i] = data_list[i][data_list[i]['LiftRotor|HoverClimb|T_to_P']<12.01]
			data_list[i] = data_list[i][data_list[i]['LiftRotor|Cruise|T_to_P']<12.01]
			data_list[i] = data_list[i][data_list[i]['LiftRotor|HoverDescent|T_to_P']<12.01]
			data_list[i] = data_list[i][data_list[i]['LiftRotor|Cruise|mu']<1.0]
			data_list[i] = data_list[i][data_list[i]['LiftRotor|Cruise|CT/sigma']<0.15]		

	elif configuration == 'liftcruise':
		for i in range(3):
			data_list[i] = data_list[i][data_list[i]['Weight|residual']<0.1]
			data_list[i] = data_list[i][data_list[i]['Aero|Cruise|CL']<0.91]
			data_list[i] = data_list[i][data_list[i]['LiftRotor|HoverClimb|T_to_P']<12.01]
			data_list[i] = data_list[i][data_list[i]['Propeller|Cruise|T_to_P']<12.01]
			data_list[i] = data_list[i][data_list[i]['LiftRotor|HoverDescent|T_to_P']<12.01]
			data_list[i] = data_list[i][data_list[i]['Propeller|Cruise|J']<3.01]
			data_list[i] = data_list[i][data_list[i]['Propeller|Cruise|CT/sigma']<0.15]
			data_list[i] = data_list[i][data_list[i]['LiftRotor|clearance_constraint']<0.1]			

	plt.plot(data_list[0]['mission_range'], data_list[0][df_label], '-o', label=f'{label} {configuration}; 250 Wh/kg')
	plt.plot(data_list[1]['mission_range'], data_list[1][df_label], '-o', label=f'{label} {configuration}; 400 Wh/kg')
	plt.plot(data_list[2]['mission_range'], data_list[2][df_label], '-o', label=f'{label} {configuration}; 550 Wh/kg')

	plt.xlabel('Mission range [km]')
	plt.ylabel(y_label)
	plt.legend()
	plt.grid()
	plt.show()

if plot_minimizing_all_objs:

	config_list = ['multirotor', 'liftcruise']
	obj_list = ['takeoff_weight', 'energy', 'mission_time']
	battery_list = ['250', '400', '550']

	data_list = []
	for config in config_list:
		for battery in battery_list:
			for obj in obj_list:
				data_list.append(pd.read_csv(f'minimizing_{obj}/{config}/battery_{battery}_Whpkg/results_with_speed_as_design_var.csv'))

	for i in range(9):
		data_list[i] = data_list[i][data_list[i]['Weight|residual']<0.1]
		data_list[i] = data_list[i][data_list[i]['LiftRotor|HoverClimb|T_to_P']<12.01]
		data_list[i] = data_list[i][data_list[i]['LiftRotor|Cruise|T_to_P']<12.01]
		data_list[i] = data_list[i][data_list[i]['LiftRotor|HoverDescent|T_to_P']<12.01]
		data_list[i] = data_list[i][data_list[i]['LiftRotor|Cruise|mu']<1.0]
		data_list[i] = data_list[i][data_list[i]['LiftRotor|Cruise|CT/sigma']<0.15]
		data_list[i]['mission_time'] *= 60
		data_list[i+9] = data_list[i+9][data_list[i+9]['Weight|residual']<0.1]
		data_list[i+9] = data_list[i+9][data_list[i+9]['Aero|Cruise|CL']<0.91]
		data_list[i+9] = data_list[i+9][data_list[i+9]['LiftRotor|HoverClimb|T_to_P']<12.01]
		data_list[i+9] = data_list[i+9][data_list[i+9]['Propeller|Cruise|T_to_P']<12.01]
		data_list[i+9] = data_list[i+9][data_list[i+9]['LiftRotor|HoverDescent|T_to_P']<12.01]
		data_list[i+9] = data_list[i+9][data_list[i+9]['Propeller|Cruise|J']<3.01]
		data_list[i+9] = data_list[i+9][data_list[i+9]['Propeller|Cruise|CT/sigma']<0.15]
		data_list[i+9] = data_list[i+9][data_list[i+9]['LiftRotor|clearance_constraint']<0.1]
		data_list[i+9]['mission_time'] *= 60

	fig, axs = plt.subplots(3, 3, figsize=(9,7), sharex=True)

	for i, battery in enumerate(battery_list):
		for j, obj in enumerate(obj_list):

			if obj == 'takeoff_weight':
				obj_ylabel = 'Takeoff weight [kg]'
				obj_dflabel = 'Weight|takeoff'
			elif obj == 'energy':
				obj_ylabel = 'Energy [kWh]'
				obj_dflabel = 'Energy|entire_mission'
			elif obj == 'mission_time':
				obj_ylabel = 'Mission time [mins]'
				obj_dflabel = 'mission_time'

			axs[j,i].plot(data_list[i*3+0]['mission_range'], data_list[i*3+0][obj_dflabel], 'r.', label=f'Weight-minimized multirotor' if i==0 and j==0 else None)
			axs[j,i].plot(data_list[i*3+1]['mission_range'], data_list[i*3+1][obj_dflabel], 'r--', label=f'Energy-minimized multirotor' if i==0 and j==0 else None)
			axs[j,i].plot(data_list[i*3+2]['mission_range'], data_list[i*3+2][obj_dflabel], 'r-', label=f'Time-minimized multirotor' if i==0 and j==0 else None)
			axs[j,i].plot(data_list[i*3+9+0]['mission_range'], data_list[i*3+9+0][obj_dflabel], 'b.', label=f'Weight-minimized lift+cruise' if i==0 and j==0 else None)
			axs[j,i].plot(data_list[i*3+9+1]['mission_range'], data_list[i*3+9+1][obj_dflabel], 'b--', label=f'Energy-minimized lift+cruise' if i==0 and j==0 else None)
			axs[j,i].plot(data_list[i*3+9+2]['mission_range'], data_list[i*3+9+2][obj_dflabel], 'b-', label=f'Time-minimized lift+cruise' if i==0 and j==0 else None)
			if i*3+j in [0,1,2]: axs[j,i].set_ylabel(obj_ylabel)
			if i*3+j in [2,5,8]: axs[j,i].set_xlabel('Mission range [km]')
			if i*3+j in [0,3,6]: axs[j,i].set_title(f'Battery {battery} Wh/kg', size=10.0)

	fig.suptitle('Single-objective optimization results')
	fig.legend(ncols=2,bbox_to_anchor=(0.8,0.96))
	# plt.tight_layout()
	plt.subplots_adjust(left=0.09, bottom=0.1, right=0.97, top=0.81, hspace=0.1)
	plt.show()
