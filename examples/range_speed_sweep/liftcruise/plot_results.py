import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

plot_2D = False
plot_contour = True
plot_surface = False
print_best_speed_for_all_range = False

plot_2D_with_speed_as_design_var = False

battery_energy_density = 250 # [250,400]

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
	print(data[['cruise_speed','Weight|takeoff','LiftRotor|radius','Propeller|Cruise|RPM']].sort_values(by='Weight|takeoff'))

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
		plt.plot(data['mission_range'], data['Weight|takeoff'], '-o', label=f'Cruise speed = {cruise_speed}')

	plt.plot(data_df2['mission_range'], data_df2['Weight|takeoff'], 'k-', label=f'Cruise speed = optimal')
	plt.xlabel('Mission range [km]')
	plt.ylabel('Optimal MTOW [kg]')
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

	# plt.plot(data_df2['mission_range'], data_df2['Weight|takeoff'], '-o', label=f'Cruise speed = optimal speed')
	plt.plot(data_df2['mission_range'], data_df2['cruise_speed'], '-o', label=f'Cruise speed = optimal speed')

	plt.xlabel('Mission range [km]')
	plt.ylabel('Optimal MTOW [kg]')
	plt.legend()
	plt.grid()
	plt.show()

if plot_contour:
	data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')	

	data_df.loc[data_df['Weight|residual']>0.1, 'Weight|takeoff'] = 10000.0
	data_df.loc[data_df['Aero|Cruise|CL']>0.91, 'Weight|takeoff'] = 10000.0
	data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P']>12.01, 'Weight|takeoff'] = 10000.0
	data_df.loc[data_df['Propeller|Cruise|T_to_P']>12.01, 'Weight|takeoff'] = 10000.0
	data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P']>12.01, 'Weight|takeoff'] = 10000.0
	data_df.loc[data_df['Propeller|Cruise|J']>3.01, 'Weight|takeoff'] = 10000.0
	data_df.loc[data_df['Propeller|Cruise|CT/sigma']>0.15, 'Weight|takeoff'] = 10000.0
	data_df.loc[data_df['LiftRotor|clearance_constraint']>0.1, 'Weight|takeoff'] = 10000.0

	range_array = np.unique(data_df['mission_range'].to_numpy())
	speed_array = np.unique(data_df['cruise_speed'].to_numpy())

	X, Y = np.meshgrid(range_array, speed_array)
	Z = data_df['Weight|takeoff'].to_numpy().reshape(len(range_array), len(speed_array)).T

	# plt.contour(X, Y, Z, 30, colors='k')
	cp = plt.contourf(X, Y, Z, 30, cmap='viridis', vmin=np.nanmin(Z), vmax=np.nanmax(Z), extend='min')
	plt.plot(data_df2['mission_range'], data_df2['cruise_speed'], 'r--', label='Optimal cruise speed path')
	plt.colorbar(cp)
	plt.ylabel('Cruise speed (km/h)')
	plt.xlabel('Mission range (km)')
	plt.title('Optimal MTOW (kg)')
	plt.legend()
	plt.show()
	# print(np.nanmin(Z))

if plot_surface:
	data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

	data_df.loc[data_df['Weight|residual']>0.1, 'Weight|takeoff'] = None
	data_df.loc[data_df['Aero|Cruise|CL']>0.91, 'Weight|takeoff'] = None
	data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P']>12.01, 'Weight|takeoff'] = None
	data_df.loc[data_df['Propeller|Cruise|T_to_P']>12.01, 'Weight|takeoff'] = None
	data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P']>12.01, 'Weight|takeoff'] = None
	data_df.loc[data_df['Propeller|Cruise|J']>3.01, 'Weight|takeoff'] = None
	data_df.loc[data_df['Propeller|Cruise|CT/sigma']>0.15, 'Weight|takeoff'] = None
	data_df.loc[data_df['LiftRotor|clearance_constraint']>0.1, 'Weight|takeoff'] = None

	data_df2.loc[data_df2['Weight|residual']>0.1, 'Weight|takeoff'] = None
	data_df2.loc[data_df2['Aero|Cruise|CL']>0.91, 'Weight|takeoff'] = None
	data_df2.loc[data_df2['LiftRotor|HoverClimb|T_to_P']>12.01, 'Weight|takeoff'] = None
	data_df2.loc[data_df2['Propeller|Cruise|T_to_P']>12.01, 'Weight|takeoff'] = None
	data_df2.loc[data_df2['LiftRotor|HoverDescent|T_to_P']>12.01, 'Weight|takeoff'] = None
	data_df2.loc[data_df2['Propeller|Cruise|J']>3.01, 'Weight|takeoff'] = None
	data_df2.loc[data_df2['Propeller|Cruise|CT/sigma']>0.15, 'Weight|takeoff'] = None
	data_df2.loc[data_df2['LiftRotor|clearance_constraint']>0.1, 'Weight|takeoff'] = None

	range_array = np.unique(data_df['mission_range'].to_numpy())
	speed_array = np.unique(data_df['cruise_speed'].to_numpy())

	X, Y = np.meshgrid(range_array, speed_array)
	Z = data_df['Weight|takeoff'].to_numpy().reshape(len(range_array), len(speed_array)).T

	Z_masked = np.ma.masked_where(Z == None, Z)

	# Create a figure and axis
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	# Plot the surface
	surf = ax.plot_surface(X, Y, Z_masked, cmap='viridis', vmin=np.nanmin(Z_masked), vmax=np.nanmax(Z_masked), antialiased=False)

	# Plot an optimal line
	ax.plot(data_df2['mission_range'], data_df2['cruise_speed'], data_df2['Weight|takeoff'], color='red', linewidth=5, label='Optimal cruise speed path')

	# Add labels and title
	ax.set_xlabel('Mission range (km)')
	ax.set_ylabel('Cruise speed (km/h)')
	ax.set_zlabel('Optimal MTOW (kg)')
	ax.set_title('Surface Plot')

	fig.colorbar(surf, orientation='horizontal', fraction=0.046, pad=0.04)

	# Show the plot
	plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.045))
	plt.show()