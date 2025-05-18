from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardSingleObjectiveOptimization
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
import numpy as np
import pandas as pd
import warnings
import sys
import matplotlib.pyplot as plt

do_optimizaton = False
do_test = False

do_plot_objective = True
do_plot_constraints = True

battery_energy_density_list = np.arange(250,725,25)
selected_battery_energy_density_list = [250,400,550]

# Fixed mission requirement
mission_range = 60 * 1609.344 # 60 miles = 96560.64 m
cruise_speed = 150 * 1609.344 / 3600 # 150 miles/hour = 67.056 m/s

solution_fidelity = {'aero':1, 'hover_climb':0}

# --- Optimization at the assumed battery technology --- #
if do_optimizaton:

	for i, battery_energy_density in enumerate(battery_energy_density_list):

		print(f'Do optimization at the assumed battery energy density = {battery_energy_density} Wh/kg')
		sys.stdout.flush() # To flush the above print output

		mtow_guess = 1000.0 # kg

		# Standard vehicle
		design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
		operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':500.0}}
		tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
		vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

		# Changed battery density
		vehicle.battery.density = float(battery_energy_density)

		# Standard mission
		mission = StandardMissionProfile(mission_range, cruise_speed)

		# Standard optimization
		with warnings.catch_warnings():
			warnings.simplefilter("ignore")
			results = {'battery_energy_density': battery_energy_density}
			results.update(RunStandardSingleObjectiveOptimization(vehicle, mission, solution_fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=False, print=True))
			results_df = pd.DataFrame(results, index=[i])
			results_df.to_csv(f'optimization_results.csv', mode='a', header=True if i==0 else False)

# --- Optimization at the assumed battery technology --- #
if do_test:

	opt_results = pd.read_csv('optimization_results.csv')

	for i, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):

		for j, battery_energy_density in  enumerate(battery_energy_density_list):

			print(f'Design optimized at {selected_battery_energy_density} Wh/kg; tested at {battery_energy_density} Wh/kg')
			sys.stdout.flush() # To flush the above print output

			wing_area = opt_results[opt_results['battery_energy_density']==selected_battery_energy_density]['Wing|area'].to_numpy()[0]
			wing_aspect_ratio = opt_results[opt_results['battery_energy_density']==selected_battery_energy_density]['Wing|aspect_ratio'].to_numpy()[0]
			r_lift_rotor = opt_results[opt_results['battery_energy_density']==selected_battery_energy_density]['LiftRotor|radius'].to_numpy()[0]
			r_propeller = opt_results[opt_results['battery_energy_density']==selected_battery_energy_density]['Propeller|radius'].to_numpy()[0]
			RPM_propeller_cruise = opt_results[opt_results['battery_energy_density']==selected_battery_energy_density]['Propeller|Cruise|RPM'].to_numpy()[0]
			cruise_speed = opt_results[opt_results['battery_energy_density']==selected_battery_energy_density]['cruise_speed'].to_numpy()[0]

			# Parasite drag of reference vehicle
			f_non_hub_non_wing = opt_results[opt_results['battery_energy_density']==selected_battery_energy_density]['Aero|Cruise|f_total_non_hub_non_wing'].to_numpy()[0]
			Cd0_wing = opt_results[opt_results['battery_energy_density']==selected_battery_energy_density]['Aero|Cruise|Cd0_wing'].to_numpy()[0]

			# Standard vehicle
			design_var = {'wing_area': wing_area, 'wing_aspect_ratio': wing_aspect_ratio, 'r_lift_rotor': r_lift_rotor, 'r_propeller': r_propeller}
			operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':RPM_propeller_cruise}}
			tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
			vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

			# Changed battery density
			vehicle.battery.density = float(battery_energy_density)

			# Fixed assumed parasite drag from the ref vehicle
			vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_non_hub_non_wing, 'descent': None}
			vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}

			# Standard mission
			mission = StandardMissionProfile(mission_range, cruise_speed*1000/3600)

			analysis = WeightAnalysis(vehicle, mission, solution_fidelity, weight_type='maximum', sizing_mode=True, solved_by='optimization')
			res = analysis.evaluate(record=False, weight_guess=2500.0, print=False)

			# Bookkeeping results
			results = {'battery_energy_density_opt':selected_battery_energy_density, 'battery_energy_density_test':battery_energy_density}

			# Mission requirements
			results['mission_range'] = mission.segments[2].distance/1000.0 	# km
			results['cruise_speed'] = res.get_val('Mission|segment_3|speed', 'km/h')[0]
			results['mission_time'] = res.get_val('Mission|total_time', 'h')[0]

			# Design objective, variables, and constraints
			results['Weight|takeoff'] = res.get_val('Weight|takeoff', 'kg')[0]
			results['Wing|area'] = res.get_val('Wing|area', 'm**2')[0]
			results['Wing|aspect_ratio'] = res.get_val('Wing|aspect_ratio', None)[0]
			results['LiftRotor|radius'] = res.get_val('LiftRotor|radius', 'm')[0]
			results['Propeller|radius'] = res.get_val('Propeller|radius', 'm')[0]
			results['Propeller|Cruise|RPM'] = res.get_val('Propeller|Cruise|RPM', 'rpm')[0]
			results['Aero|Cruise|CL'] = res.get_val('Aero|Cruise|CL', None)[0]
			results['Propeller|Cruise|CT/sigma'] = res.get_val('Propeller|Cruise|thrust_coefficient')[0]/vehicle.propeller.solidity
			results['Propeller|Cruise|J'] = res.get_val('Propeller|Cruise|J')[0]
			results['LiftRotor|HoverClimb|T_to_P'] = res.get_val('LiftRotor|HoverClimb|T_to_P')[0]
			results['Propeller|Cruise|T_to_P'] = res.get_val('Propeller|Cruise|T_to_P')[0]
			results['LiftRotor|HoverDescent|T_to_P'] = res.get_val('LiftRotor|HoverDescent|T_to_P')[0]
			results['LiftRotor|clearance_constraint'] = res.get_val('LiftRotor|clearance_constraint')[0]
			results['Weight|residual'] = res.get_val('Weight|residual', 'kg')[0]

			# Other importants results
			results['Weight|payload'] = res.get_val('Weight|payload', 'kg')[0]
			results['Weight|battery'] = res.get_val('Weight|battery', 'kg')[0]
			results['Weight|propulsion'] = res.get_val('Weight|propulsion', 'kg')[0]
			results['Weight|structure'] = res.get_val('Weight|structure', 'kg')[0]
			results['Weight|equipment'] = res.get_val('Weight|equipment', 'kg')[0]
			results['Power|segment_1'] = res.get_val('Power|segment_1', 'kW')[0]
			results['Power|segment_3'] = res.get_val('Power|segment_3', 'kW')[0]
			results['Power|segment_5'] = res.get_val('Power|segment_5', 'kW')[0]
			results['DiskLoading|LiftRotor|segment_1'] = res.get_val('DiskLoading|LiftRotor|segment_1', 'N/m**2')[0]
			results['DiskLoading|Propeller|segment_3'] = res.get_val('DiskLoading|Propeller|segment_3', 'N/m**2')[0]
			results['DiskLoading|LiftRotor|segment_5'] = res.get_val('DiskLoading|LiftRotor|segment_5', 'N/m**2')[0]
			results['Energy|entire_mission'] = res.get_val('Energy|entire_mission', 'kW*h')[0]

			# Aerodynamics at cruise
			results['Aero|Cruise|f_total_non_hub_non_wing'] = vehicle.f_total_non_hub_non_wing['cruise']
			results['Aero|Cruise|Cd0_wing'] = vehicle.wing.Cd0['cruise']
			results['Aero|Cruise|f_total'] = res.get_val('Aero|Cruise|f_total', 'm**2')[0]
			results['Aero|Cruise|total_drag'] = res.get_val('Aero|Cruise|total_drag', 'N')[0]

			# Saving to csv file
			results_df = pd.DataFrame(results, index=[i*3+j])
			results_df.to_csv(f'test_results.csv', mode='a', header=True if i*3+j==0 else False)

if do_plot_objective:
	opt_results = pd.read_csv('optimization_results.csv')
	test_results = pd.read_csv('test_results.csv')
	# ylabel = 'Energy|entire_mission'
	ylabel = 'Weight|takeoff'

	# Processing constraints
	opt_results.loc[opt_results['Weight|residual']>0.1, ylabel] = np.nan
	opt_results.loc[opt_results['Aero|Cruise|CL']>0.91, ylabel] = np.nan
	opt_results.loc[opt_results['LiftRotor|HoverClimb|T_to_P']>12.01, ylabel] = np.nan
	opt_results.loc[opt_results['Propeller|Cruise|T_to_P']>12.01, ylabel] = np.nan
	opt_results.loc[opt_results['LiftRotor|HoverDescent|T_to_P']>12.01, ylabel] = np.nan
	opt_results.loc[opt_results['Propeller|Cruise|J']>3.01, ylabel] = np.nan
	opt_results.loc[opt_results['Propeller|Cruise|CT/sigma']>0.141, ylabel] = np.nan
	opt_results.loc[opt_results['LiftRotor|clearance_constraint']>0.1, ylabel] = np.nan

	test_results.loc[test_results['Weight|residual']>0.1, ylabel] = np.nan
	test_results.loc[test_results['Aero|Cruise|CL']>0.91, ylabel] = np.nan
	test_results.loc[test_results['LiftRotor|HoverClimb|T_to_P']>12.01, ylabel] = np.nan
	test_results.loc[test_results['Propeller|Cruise|T_to_P']>12.01, ylabel] = np.nan
	test_results.loc[test_results['LiftRotor|HoverDescent|T_to_P']>12.01, ylabel] = np.nan
	test_results.loc[test_results['Propeller|Cruise|J']>3.01, ylabel] = np.nan
	test_results.loc[test_results['Propeller|Cruise|CT/sigma']>0.141, ylabel] = np.nan
	test_results.loc[test_results['LiftRotor|clearance_constraint']>0.1, ylabel] = np.nan

	# print(opt_results[ylabel])

	for i, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
		test_results_i = test_results[test_results['battery_energy_density_opt']==selected_battery_energy_density][ylabel]
		plt.plot(battery_energy_density_list, test_results_i, 'o-', ms=4.0, label=f'Optimized at {selected_battery_energy_density_list[i]} Wh/kg')
	plt.plot(opt_results['battery_energy_density'], opt_results[ylabel], 'k--', label=f'Optimized at test battery GED')	
	plt.title('Suboptimal performance evaluation\nfor Lift+Cruise from weight-based optimization')
	plt.xlabel(r'Test battery GED $[Wh/kg]$')
	plt.ylabel(r'Takeoff weight $[kg]$')
	plt.legend()
	plt.show()

if do_plot_constraints:
	test_results = pd.read_csv('test_results.csv')
	xmin = battery_energy_density_list[0]
	xmax = battery_energy_density_list[-1]

	ylabels = ['Aero|Cruise|CL','Propeller|Cruise|CT/sigma','Propeller|Cruise|J','Weight|residual',
			   'LiftRotor|HoverClimb|T_to_P','Propeller|Cruise|T_to_P','LiftRotor|HoverDescent|T_to_P','LiftRotor|clearance_constraint']
	ylabel_names = [r'Lift coeff $C_{L}$',r'Prop blade loading $CT/\sigma$',r'Prop advance ratio $J$',r'Squared residual $[kg^2]$',
					r'$T/P$ HoverClimb $[g/W]$',r'$T/P$ Cruise $[g/W]$',r'$T/P$ HoverDescent $[g/W]$',r'Clearance constraint $[m]$']

	fig, axes = plt.subplots(2, 4, sharex=True, figsize=(14,7))
	axes = axes.flatten()

	for j, ylabel in enumerate(ylabels):
		for i, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):

			test_results_i = test_results[test_results['battery_energy_density_opt']==selected_battery_energy_density][ylabel]
			axes[j].plot(battery_energy_density_list, test_results_i, 'o-', ms=3.0, label=f'Optimized at {selected_battery_energy_density} Wh/kg' if j==0 else None)
			
			if ylabel == 'Aero|Cruise|CL': axes[j].hlines(0.9,xmin=xmin,xmax=xmax,linestyles='--',color='k')
			if ylabel == 'Propeller|Cruise|CT/sigma': axes[j].hlines(0.14,xmin=xmin,xmax=xmax,linestyles='--',color='k',label='Constraint upper limit' if i==2 else None)
			if ylabel == 'Propeller|Cruise|J': axes[j].hlines(3.001,xmin=xmin,xmax=xmax,linestyles='--',color='k')
			if ylabel == 'LiftRotor|HoverClimb|T_to_P': axes[j].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
			if ylabel == 'Propeller|Cruise|T_to_P': axes[j].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
			if ylabel == 'LiftRotor|HoverDescent|T_to_P': axes[j].hlines(12.0,xmin=xmin,xmax=xmax,linestyles='--',color='k')
			if ylabel == 'LiftRotor|clearance_constraint': axes[j].hlines(1e-5,xmin=xmin,xmax=xmax,linestyles='--',color='k')

			if j in [4,5,6,7]: axes[j].set_xlabel('Test battery GED [Wh/kg]')
			axes[j].set_ylabel(ylabel_names[j])
	axes[3].ticklabel_format(style='sci', axis='y', scilimits=(0,0))
	fig.suptitle('Constraint evaluation with varying battery GED for Lift+Cruise from weight-based optimization')
	fig.legend(ncols=2,bbox_to_anchor=(0.50,0.95),loc='upper center')
	# plt.subplots_adjust(left=0.07, bottom=0.1, right=0.98, top=0.83, hspace=0.08)
	plt.tight_layout(rect=(0,0,1,0.93))
	plt.show()


