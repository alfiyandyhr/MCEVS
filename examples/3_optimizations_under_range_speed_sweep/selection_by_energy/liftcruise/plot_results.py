import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

plot_2D = False
plot_contour = False
plot_contour_all = True; savefig = False
plot_surface = False

print_best_speed_for_all_range = False
check_best_speed_for_all_range = False

plot_2D_with_speed_as_design_var = False

battery_energy_density = 250 # [250,400,550]
battery_energy_density_list = [250,400,550]

if print_best_speed_for_all_range:
	data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
	data_df = data_df[data_df['Weight|residual']<0.1]
	data_df = data_df[data_df['Aero|Cruise|CL']<0.91]
	data_df = data_df[data_df['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df = data_df[data_df['Propeller|Cruise|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df = data_df[data_df['Propeller|Cruise|J']<3.01]
	data_df = data_df[data_df['Propeller|Cruise|CT/sigma']<0.15]
	data_df = data_df[data_df['LiftRotor|clearance_constraint']<0.1]
	data = data_df[data_df['mission_range']==10]
	print(data[['cruise_speed','Energy|entire_mission','LiftRotor|radius','Propeller|Cruise|RPM']].sort_values(by='Energy|entire_mission'))

if check_best_speed_for_all_range:
	data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
	data_df = data_df[data_df['Weight|residual']<0.1]
	data_df = data_df[data_df['Aero|Cruise|CL']<0.91]
	data_df = data_df[data_df['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df = data_df[data_df['Propeller|Cruise|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df = data_df[data_df['Propeller|Cruise|J']<3.01]
	data_df = data_df[data_df['Propeller|Cruise|CT/sigma']<0.15]
	data_df = data_df[data_df['LiftRotor|clearance_constraint']<0.1]
	data = data_df[data_df['mission_range']==10]
	
	opt_speed_data = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')
	
	for mission_range in range(10,230,10):
		weight_at_opt_speed = opt_speed_data[opt_speed_data['mission_range']==mission_range]['Energy|entire_mission'].to_numpy()[0]
		data = data_df[data_df['mission_range']==mission_range]
		weight_at_best_speed = data[['cruise_speed','Energy|entire_mission']].sort_values(by='Energy|entire_mission').iloc[0]['Energy|entire_mission']
		status = 'passed!' if weight_at_opt_speed < weight_at_best_speed else 'NOT passed!'
		print(f'mission_range = {mission_range}; weight_at_opt_speed = {weight_at_opt_speed}; weight_at_best_speed = {weight_at_best_speed}; status = {status}')

if plot_2D:
	data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

	data_df = data_df[data_df['Weight|residual']<0.1]
	data_df = data_df[data_df['Aero|Cruise|CL']<0.91]
	data_df = data_df[data_df['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df = data_df[data_df['Propeller|Cruise|T_to_P']<12.01]
	data_df = data_df[data_df['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df = data_df[data_df['Propeller|Cruise|J']<3.01]
	data_df = data_df[data_df['Propeller|Cruise|CT/sigma']<0.15]
	data_df = data_df[data_df['LiftRotor|clearance_constraint']<0.1]

	data_df2 = data_df2[data_df2['Weight|residual']<0.1]
	data_df2 = data_df2[data_df2['Aero|Cruise|CL']<0.91]
	data_df2 = data_df2[data_df2['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|J']<3.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|CT/sigma']<0.15]
	data_df2 = data_df2[data_df2['LiftRotor|clearance_constraint']<0.1]

	range_array = np.unique(data_df['mission_range'].to_numpy())
	speed_array = np.unique(data_df['cruise_speed'].to_numpy())

	for cruise_speed in speed_array:
		data = data_df[data_df['cruise_speed']==cruise_speed]
		plt.plot(data['mission_range'], data['Energy|entire_mission'], '-o', label=f'Cruise speed = {cruise_speed}')

	plt.plot(data_df2['mission_range'], data_df2['Energy|entire_mission'], 'k-o', label=f'Cruise speed = optimal')
	plt.xlabel('Mission range [km]')
	plt.ylabel('Optimal energy [kWh]')
	plt.legend()
	plt.grid()
	plt.show()

if plot_2D_with_speed_as_design_var:
	data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

	data_df2 = data_df2[data_df2['Weight|residual']<0.1]
	data_df2 = data_df2[data_df2['Aero|Cruise|CL']<0.91]
	data_df2 = data_df2[data_df2['LiftRotor|HoverClimb|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['LiftRotor|HoverDescent|T_to_P']<12.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|J']<3.01]
	data_df2 = data_df2[data_df2['Propeller|Cruise|CT/sigma']<0.15]
	data_df2 = data_df2[data_df2['LiftRotor|clearance_constraint']<0.1]

	plt.plot(data_df2['mission_range'], data_df2['Energy|entire_mission'], '-o', label=f'Cruise speed = optimal speed')
	# plt.plot(data_df2['mission_range'], data_df2['cruise_speed'], '-o', label=f'Cruise speed = optimal speed')

	plt.xlabel('Mission range [km]')
	plt.ylabel('Optimal energy [kWh]')
	plt.legend()
	plt.grid()
	plt.show()

if plot_contour:
	data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')	

	data_df.loc[data_df['Weight|residual']>0.1, 'Energy|entire_mission'] = 2000.0
	data_df.loc[data_df['Aero|Cruise|CL']>0.91, 'Energy|entire_mission'] = 2000.0
	data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P']>12.01, 'Energy|entire_mission'] = 2000.0
	data_df.loc[data_df['Propeller|Cruise|T_to_P']>12.01, 'Energy|entire_mission'] = 2000.0
	data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P']>12.01, 'Energy|entire_mission'] = 2000.0
	data_df.loc[data_df['Propeller|Cruise|J']>3.01, 'Energy|entire_mission'] = 2000.0
	data_df.loc[data_df['Propeller|Cruise|CT/sigma']>0.15, 'Energy|entire_mission'] = 2000.0
	data_df.loc[data_df['LiftRotor|clearance_constraint']>0.1, 'Energy|entire_mission'] = 2000.0

	range_array = np.unique(data_df['mission_range'].to_numpy())
	speed_array = np.unique(data_df['cruise_speed'].to_numpy())

	X, Y = np.meshgrid(range_array, speed_array)
	Z = data_df['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T

	# plt.contour(X, Y, Z, 30, colors='k')
	cp = plt.contourf(X, Y, Z, 30, cmap='viridis', vmin=np.nanmin(Z), vmax=np.nanmax(Z), extend='min')
	plt.plot(data_df2['mission_range'], data_df2['cruise_speed'], 'r--', label='Optimal cruise speed path')
	plt.colorbar(cp)
	plt.ylabel('Cruise speed (km/h)')
	plt.xlabel('Mission range (km)')
	plt.title('Optimal energy (kWh)')
	plt.legend()
	plt.show()
	# print(np.nanmin(Z))

if plot_surface:
	data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

	data_df.loc[data_df['Weight|residual']>0.1, 'Energy|entire_mission'] = None
	data_df.loc[data_df['Aero|Cruise|CL']>0.91, 'Energy|entire_mission'] = None
	data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P']>12.01, 'Energy|entire_mission'] = None
	data_df.loc[data_df['Propeller|Cruise|T_to_P']>12.01, 'Energy|entire_mission'] = None
	data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P']>12.01, 'Energy|entire_mission'] = None
	data_df.loc[data_df['Propeller|Cruise|J']>3.01, 'Energy|entire_mission'] = None
	data_df.loc[data_df['Propeller|Cruise|CT/sigma']>0.15, 'Energy|entire_mission'] = None
	data_df.loc[data_df['LiftRotor|clearance_constraint']>0.1, 'Energy|entire_mission'] = None

	data_df2.loc[data_df2['Weight|residual']>0.1, 'Energy|entire_mission'] = None
	data_df2.loc[data_df2['Aero|Cruise|CL']>0.91, 'Energy|entire_mission'] = None
	data_df2.loc[data_df2['LiftRotor|HoverClimb|T_to_P']>12.01, 'Energy|entire_mission'] = None
	data_df2.loc[data_df2['Propeller|Cruise|T_to_P']>12.01, 'Energy|entire_mission'] = None
	data_df2.loc[data_df2['LiftRotor|HoverDescent|T_to_P']>12.01, 'Energy|entire_mission'] = None
	data_df2.loc[data_df2['Propeller|Cruise|J']>3.01, 'Energy|entire_mission'] = None
	data_df2.loc[data_df2['Propeller|Cruise|CT/sigma']>0.15, 'Energy|entire_mission'] = None
	data_df2.loc[data_df2['LiftRotor|clearance_constraint']>0.1, 'Energy|entire_mission'] = None

	range_array = np.unique(data_df['mission_range'].to_numpy())
	speed_array = np.unique(data_df['cruise_speed'].to_numpy())

	X, Y = np.meshgrid(range_array, speed_array)
	Z = data_df['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T

	Z_masked = np.ma.masked_where(Z == None, Z)

	# Create a figure and axis
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	# Plot the surface
	surf = ax.plot_surface(X, Y, Z_masked, cmap='viridis', vmin=np.nanmin(Z_masked), vmax=np.nanmax(Z_masked), antialiased=False)

	# Plot an optimal line
	ax.plot(data_df2['mission_range'], data_df2['cruise_speed'], data_df2['Energy|entire_mission'], color='red', linewidth=5, label='Optimal cruise speed path')

	# Add labels and title
	ax.set_xlabel('Mission range (km)')
	ax.set_ylabel('Cruise speed (km/h)')
	ax.set_zlabel('Optimal MTOW (kg)')
	ax.set_title('Surface Plot')

	fig.colorbar(surf, orientation='horizontal', fraction=0.046, pad=0.04)

	# Show the plot
	plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.045))
	plt.show()

if plot_contour_all:
	# Create a figure and subplots
	fig, axes = plt.subplots(1, 3, figsize=(14, 6), sharey=True)  # 1 row, 3 columns of subplots

	for i, battery_energy_density in enumerate(battery_energy_density_list):

		# Read data for the current battery energy density
		data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
		data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

		# Apply filtering conditions
		data_df.loc[data_df['Weight|residual']>0.1, 'Energy|entire_mission'] = 500.0
		data_df.loc[data_df['Aero|Cruise|CL']>0.91, 'Energy|entire_mission'] = 500.0
		data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P']>12.01, 'Energy|entire_mission'] = 500.0
		data_df.loc[data_df['Propeller|Cruise|T_to_P']>12.01, 'Energy|entire_mission'] = 500.0
		data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P']>12.01, 'Energy|entire_mission'] = 500.0
		data_df.loc[data_df['Propeller|Cruise|J']>3.01, 'Energy|entire_mission'] = 500.0
		data_df.loc[data_df['Propeller|Cruise|CT/sigma']>0.15, 'Energy|entire_mission'] = 500.0
		data_df.loc[data_df['LiftRotor|clearance_constraint']>0.1, 'Energy|entire_mission'] = 500.0

		# Prepare data for contour plot
		range_array = np.unique(data_df['mission_range'].to_numpy())
		speed_array = np.unique(data_df['cruise_speed'].to_numpy())

		X, Y = np.meshgrid(range_array, speed_array)
		Z = data_df['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T

		# Plot in the corresponding subplot
		ax = axes[i]
		cp = ax.contourf(X, Y, Z, 30, cmap='viridis', vmin=np.nanmin(Z), vmax=np.nanmax(Z), extend='both')
		ax.plot(data_df2['mission_range'], data_df2['cruise_speed'], 'r--', label='Optimal cruise speed path')
		ax.set_title(f'Battery GED: {battery_energy_density} Wh/kg')
		ax.set_xlabel('Mission range (km)')
		if i == 0: ax.set_ylabel('Cruise speed (km/h)')
		# ax.legend()

	# Add a single legend for the "Optimal cruise speed path" line
	fig.legend(['Optimal cruise speed path'], loc='upper center', bbox_to_anchor=(0.49, 0.94), ncol=1, prop={'size': 12})

	# Add a horizontal colorbar below the subplots
	cbar = fig.colorbar(cp, ax=axes, orientation='horizontal', fraction=0.05, pad=0.1)
	cbar.set_label('Optimal energy (kWh)')

	# Adjust layout to prevent overlap
	fig.suptitle('Energy-based optimization results for Lift+Cruise', size=16)
	plt.subplots_adjust(bottom=0.25, top=0.82, wspace=0.03) 
	plt.savefig('energy_opt_range_speed_space_liftcruise.pdf', format='pdf', dpi=300) if savefig else plt.show()
