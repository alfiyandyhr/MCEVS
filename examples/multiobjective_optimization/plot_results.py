import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

plot_minimizing_takeoff_weight = False
plot_minimizing_energy = False
plot_minimizing_mission_time = False

plot_minimizing_all_objs = True

configuration = 'multirotor'
# configuration = 'liftcruise'

if plot_minimizing_takeoff_weight:
	data_df1 = pd.read_csv(f'minimizing_takeoff_weight/{configuration}/battery_250_Whpkg/results_with_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'minimizing_takeoff_weight/{configuration}/battery_400_Whpkg/results_with_speed_as_design_var.csv')
	data_df3 = pd.read_csv(f'minimizing_takeoff_weight/{configuration}/battery_550_Whpkg/results_with_speed_as_design_var.csv')
	
	data1 = data_df1[data_df1['Weight|residual']<0.1]
	data1 = data1[data1['LiftRotor|HoverClimb|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|Cruise|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|HoverDescent|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|Cruise|mu']<1.0]
	data1 = data1[data1['LiftRotor|Cruise|CT/sigma']<0.15]

	data2 = data_df2[data_df2['Weight|residual']<0.1]
	data2 = data2[data2['LiftRotor|HoverClimb|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|Cruise|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|HoverDescent|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|Cruise|mu']<1.0]
	data2 = data2[data2['LiftRotor|Cruise|CT/sigma']<0.15]

	data3 = data_df3[data_df3['Weight|residual']<0.1]
	data3 = data3[data3['LiftRotor|HoverClimb|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|Cruise|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|HoverDescent|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|Cruise|mu']<1.0]
	data3 = data3[data3['LiftRotor|Cruise|CT/sigma']<0.15]

	plt.plot(data1['mission_range'], data1['Weight|takeoff'], '-o', label=f'Weight-minimized {configuration}; 250 Wh/kg')
	plt.plot(data2['mission_range'], data2['Weight|takeoff'], '-o', label=f'Weight-minimized {configuration}; 400 Wh/kg')
	plt.plot(data3['mission_range'], data3['Weight|takeoff'], '-o', label=f'Weight-minimized {configuration}; 550 Wh/kg')

	plt.xlabel('Mission range [km]')
	plt.ylabel('Takeoff weight [kg]')
	plt.legend()
	plt.grid()
	plt.show()

if plot_minimizing_energy:
	data_df1 = pd.read_csv(f'minimizing_energy/{configuration}/battery_250_Whpkg/results_with_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'minimizing_energy/{configuration}/battery_400_Whpkg/results_with_speed_as_design_var.csv')
	data_df3 = pd.read_csv(f'minimizing_energy/{configuration}/battery_550_Whpkg/results_with_speed_as_design_var.csv')

	data1 = data_df1[data_df1['Weight|residual']<0.1]
	data1 = data1[data1['LiftRotor|HoverClimb|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|Cruise|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|HoverDescent|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|Cruise|mu']<1.0]
	data1 = data1[data1['LiftRotor|Cruise|CT/sigma']<0.15]

	data2 = data_df2[data_df2['Weight|residual']<0.1]
	data2 = data2[data2['LiftRotor|HoverClimb|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|Cruise|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|HoverDescent|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|Cruise|mu']<1.0]
	data2 = data2[data2['LiftRotor|Cruise|CT/sigma']<0.15]

	data3 = data_df3[data_df3['Weight|residual']<0.1]
	data3 = data3[data3['LiftRotor|HoverClimb|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|Cruise|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|HoverDescent|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|Cruise|mu']<1.0]
	data3 = data3[data3['LiftRotor|Cruise|CT/sigma']<0.15]

	plt.plot(data1['mission_range'], data1['Energy|entire_mission'], '-o', label=f'Energy-minimized {configuration}; 250 Wh/kg')
	plt.plot(data2['mission_range'], data2['Energy|entire_mission'], '-o', label=f'Energy-minimized {configuration}; 400 Wh/kg')
	plt.plot(data3['mission_range'], data3['Energy|entire_mission'], '-o', label=f'Energy-minimized {configuration}; 550 Wh/kg')

	plt.xlabel('Mission range [km]')
	plt.ylabel('Energy [kWh]')
	plt.legend()
	plt.grid()
	plt.show()

if plot_minimizing_mission_time:
	data_df1 = pd.read_csv(f'minimizing_mission_time/{configuration}/battery_250_Whpkg/results_with_speed_as_design_var.csv')
	data_df2 = pd.read_csv(f'minimizing_mission_time/{configuration}/battery_400_Whpkg/results_with_speed_as_design_var.csv')
	data_df3 = pd.read_csv(f'minimizing_mission_time/{configuration}/battery_550_Whpkg/results_with_speed_as_design_var.csv')
	
	data1 = data_df1[data_df1['Weight|residual']<0.1]
	data1 = data1[data1['LiftRotor|HoverClimb|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|Cruise|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|HoverDescent|T_to_P']<12.01]
	data1 = data1[data1['LiftRotor|Cruise|mu']<1.0]
	data1 = data1[data1['LiftRotor|Cruise|CT/sigma']<0.15]

	data2 = data_df2[data_df2['Weight|residual']<0.1]
	data2 = data2[data2['LiftRotor|HoverClimb|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|Cruise|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|HoverDescent|T_to_P']<12.01]
	data2 = data2[data2['LiftRotor|Cruise|mu']<1.0]
	data2 = data2[data2['LiftRotor|Cruise|CT/sigma']<0.15]

	data3 = data_df3[data_df3['Weight|residual']<0.1]
	data3 = data3[data3['LiftRotor|HoverClimb|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|Cruise|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|HoverDescent|T_to_P']<12.01]
	data3 = data3[data3['LiftRotor|Cruise|mu']<1.0]
	data3 = data3[data3['LiftRotor|Cruise|CT/sigma']<0.15]

	plt.plot(data1['mission_range'], data1['Weight|takeoff'], '-o', label=f'Time-minimized {configuration}; 250 Wh/kg')
	plt.plot(data2['mission_range'], data2['Weight|takeoff'], '-o', label=f'Time-minimized {configuration}; 400 Wh/kg')
	plt.plot(data3['mission_range'], data3['Weight|takeoff'], '-o', label=f'Time-minimized {configuration}; 550 Wh/kg')

	plt.xlabel('Mission range [km]')
	plt.ylabel('Takeoff weight [kg]')
	plt.legend()
	plt.grid()
	plt.show()

if plot_minimizing_all_objs:
	data_df1a = pd.read_csv(f'minimizing_takeoff_weight/{configuration}/battery_250_Whpkg/results_with_speed_as_design_var.csv')
	data_df2a = pd.read_csv(f'minimizing_energy/{configuration}/battery_250_Whpkg/results_with_speed_as_design_var.csv')
	data_df3a = pd.read_csv(f'minimizing_mission_time/{configuration}/battery_250_Whpkg/results_with_speed_as_design_var.csv')
	
	data_df1b = pd.read_csv(f'minimizing_takeoff_weight/{configuration}/battery_400_Whpkg/results_with_speed_as_design_var.csv')
	data_df2b = pd.read_csv(f'minimizing_energy/{configuration}/battery_400_Whpkg/results_with_speed_as_design_var.csv')
	data_df3b = pd.read_csv(f'minimizing_mission_time/{configuration}/battery_400_Whpkg/results_with_speed_as_design_var.csv')
	
	data_df1c = pd.read_csv(f'minimizing_takeoff_weight/{configuration}/battery_550_Whpkg/results_with_speed_as_design_var.csv')
	data_df2c = pd.read_csv(f'minimizing_energy/{configuration}/battery_550_Whpkg/results_with_speed_as_design_var.csv')
	data_df3c = pd.read_csv(f'minimizing_mission_time/{configuration}/battery_550_Whpkg/results_with_speed_as_design_var.csv')
	

	data1a = data_df1a[data_df1a['Weight|residual']<0.1]
	data1a = data1a[data1a['LiftRotor|HoverClimb|T_to_P']<12.01]
	data1a = data1a[data1a['LiftRotor|Cruise|T_to_P']<12.01]
	data1a = data1a[data1a['LiftRotor|HoverDescent|T_to_P']<12.01]
	data1a = data1a[data1a['LiftRotor|Cruise|mu']<1.0]
	data1a = data1a[data1a['LiftRotor|Cruise|CT/sigma']<0.15]

	data2a = data_df2a[data_df2a['Weight|residual']<0.1]
	data2a = data2a[data2a['LiftRotor|HoverClimb|T_to_P']<12.01]
	data2a = data2a[data2a['LiftRotor|Cruise|T_to_P']<12.01]
	data2a = data2a[data2a['LiftRotor|HoverDescent|T_to_P']<12.01]
	data2a = data2a[data2a['LiftRotor|Cruise|mu']<1.0]
	data2a = data2a[data2a['LiftRotor|Cruise|CT/sigma']<0.15]

	data3a = data_df3a[data_df3a['Weight|residual']<0.1]
	data3a = data3a[data3a['LiftRotor|HoverClimb|T_to_P']<12.01]
	data3a = data3a[data3a['LiftRotor|Cruise|T_to_P']<12.01]
	data3a = data3a[data3a['LiftRotor|HoverDescent|T_to_P']<12.01]
	data3a = data3a[data3a['LiftRotor|Cruise|mu']<1.0]
	data3a = data3a[data3a['LiftRotor|Cruise|CT/sigma']<0.15]

	data1b = data_df1b[data_df1b['Weight|residual']<0.1]
	data1b = data1b[data1b['LiftRotor|HoverClimb|T_to_P']<12.01]
	data1b = data1b[data1b['LiftRotor|Cruise|T_to_P']<12.01]
	data1b = data1b[data1b['LiftRotor|HoverDescent|T_to_P']<12.01]
	data1b = data1b[data1b['LiftRotor|Cruise|mu']<1.0]
	data1b = data1b[data1b['LiftRotor|Cruise|CT/sigma']<0.15]

	data2b = data_df2b[data_df2b['Weight|residual']<0.1]
	data2b = data2b[data2b['LiftRotor|HoverClimb|T_to_P']<12.01]
	data2b = data2b[data2b['LiftRotor|Cruise|T_to_P']<12.01]
	data2b = data2b[data2b['LiftRotor|HoverDescent|T_to_P']<12.01]
	data2b = data2b[data2b['LiftRotor|Cruise|mu']<1.0]
	data2b = data2b[data2b['LiftRotor|Cruise|CT/sigma']<0.15]

	data3b = data_df3b[data_df3b['Weight|residual']<0.1]
	data3b = data3b[data3b['LiftRotor|HoverClimb|T_to_P']<12.01]
	data3b = data3b[data3b['LiftRotor|Cruise|T_to_P']<12.01]
	data3b = data3b[data3b['LiftRotor|HoverDescent|T_to_P']<12.01]
	data3b = data3b[data3b['LiftRotor|Cruise|mu']<1.0]
	data3b = data3b[data3b['LiftRotor|Cruise|CT/sigma']<0.15]

	data1c = data_df1c[data_df1c['Weight|residual']<0.1]
	data1c = data1c[data1c['LiftRotor|HoverClimb|T_to_P']<12.01]
	data1c = data1c[data1c['LiftRotor|Cruise|T_to_P']<12.01]
	data1c = data1c[data1c['LiftRotor|HoverDescent|T_to_P']<12.01]
	data1c = data1c[data1c['LiftRotor|Cruise|mu']<1.0]
	data1c = data1c[data1c['LiftRotor|Cruise|CT/sigma']<0.15]

	data2c = data_df2c[data_df2c['Weight|residual']<0.1]
	data2c = data2c[data2c['LiftRotor|HoverClimb|T_to_P']<12.01]
	data2c = data2c[data2c['LiftRotor|Cruise|T_to_P']<12.01]
	data2c = data2c[data2c['LiftRotor|HoverDescent|T_to_P']<12.01]
	data2c = data2c[data2c['LiftRotor|Cruise|mu']<1.0]
	data2c = data2c[data2c['LiftRotor|Cruise|CT/sigma']<0.15]

	data3c = data_df3c[data_df3c['Weight|residual']<0.1]
	data3c = data3c[data3c['LiftRotor|HoverClimb|T_to_P']<12.01]
	data3c = data3c[data3c['LiftRotor|Cruise|T_to_P']<12.01]
	data3c = data3c[data3c['LiftRotor|HoverDescent|T_to_P']<12.01]
	data3c = data3c[data3c['LiftRotor|Cruise|mu']<1.0]
	data3c = data3c[data3c['LiftRotor|Cruise|CT/sigma']<0.15]

	fig, axs = plt.subplots(3, 3, figsize=(9,7), sharex=True)

	axs[0,0].plot(data1a['mission_range'], data1a['Weight|takeoff'], 'r.', label=f'Weight-minimized {configuration}')
	axs[0,0].plot(data2a['mission_range'], data2a['Weight|takeoff'], 'r--', label=f'Energy-minimized {configuration}')
	axs[0,0].plot(data3a['mission_range'], data3a['Weight|takeoff'], 'r-', label=f'Time-minimized {configuration}')
	axs[0,0].set_ylabel('Takeoff weight [kg]')
	axs[0,0].set_title('Battery 250 Wh/kg', size=10.0)

	axs[1,0].plot(data1a['mission_range'], data1a['Energy|entire_mission'], 'r.')
	axs[1,0].plot(data2a['mission_range'], data2a['Energy|entire_mission'], 'r--')
	axs[1,0].plot(data3a['mission_range'], data3a['Energy|entire_mission'], 'r-')
	axs[1,0].set_ylabel('Energy [kWh]')

	axs[2,0].plot(data1a['mission_range'], data1a['mission_time']*60, 'r.')
	axs[2,0].plot(data2a['mission_range'], data2a['mission_time']*60, 'r--')
	axs[2,0].plot(data3a['mission_range'], data3a['mission_time']*60, 'r-')
	axs[2,0].set_ylabel('Mission time [mins]')
	axs[2,0].set_xlabel('Mission range [km]')

	axs[0,1].plot(data1b['mission_range'], data1b['Weight|takeoff'], 'r.')
	axs[0,1].plot(data2b['mission_range'], data2b['Weight|takeoff'], 'r--')
	axs[0,1].plot(data3b['mission_range'], data3b['Weight|takeoff'], 'r-')
	axs[0,1].set_title('Battery 400 Wh/kg', size=10.0)

	axs[1,1].plot(data1b['mission_range'], data1b['Energy|entire_mission'], 'r.')
	axs[1,1].plot(data2b['mission_range'], data2b['Energy|entire_mission'], 'r--')
	axs[1,1].plot(data3b['mission_range'], data3b['Energy|entire_mission'], 'r-')

	axs[2,1].plot(data1b['mission_range'], data1b['mission_time']*60, 'r.')
	axs[2,1].plot(data2b['mission_range'], data2b['mission_time']*60, 'r--')
	axs[2,1].plot(data3b['mission_range'], data3b['mission_time']*60, 'r-')
	axs[2,1].set_xlabel('Mission range [km]')

	axs[0,2].plot(data1c['mission_range'], data1c['Weight|takeoff'], 'r.')
	axs[0,2].plot(data2c['mission_range'], data2c['Weight|takeoff'], 'r--')
	axs[0,2].plot(data3c['mission_range'], data3c['Weight|takeoff'], 'r-')
	axs[0,2].set_title('Battery 550 Wh/kg', size=10.0)

	axs[1,2].plot(data1c['mission_range'], data1c['Energy|entire_mission'], 'r.')
	axs[1,2].plot(data2c['mission_range'], data2c['Energy|entire_mission'], 'r--')
	axs[1,2].plot(data3c['mission_range'], data3c['Energy|entire_mission'], 'r-')

	axs[2,2].plot(data1c['mission_range'], data1c['mission_time']*60, 'r.')
	axs[2,2].plot(data2c['mission_range'], data2c['mission_time']*60, 'r--')
	axs[2,2].plot(data3c['mission_range'], data3c['mission_time']*60, 'r-')
	axs[2,2].set_xlabel('Mission range [km]')

	fig.suptitle('Single-objective optimization results')
	fig.legend(bbox_to_anchor=(0.5,0.96))
	# plt.tight_layout()
	plt.subplots_adjust(left=0.09, bottom=0.1, right=0.97, top=0.81, hspace=0.1)
	plt.show()
