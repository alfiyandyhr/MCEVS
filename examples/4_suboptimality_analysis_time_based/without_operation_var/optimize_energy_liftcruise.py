from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardSingleObjectiveOptimization, RunOffDesignSingleObjectiveOptimization
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
from MCEVS.Optimization.Container import DesignProblem
import numpy as np
import pandas as pd
import warnings
import sys
import matplotlib.pyplot as plt

produce_utopian_data = False
evaluate_selected_designs = False; selected_years = [2030, 2050]

plot_utopian_data = False
plot_evaluation_data = True

# 40 year lifespan of the product
year_list = np.arange(2030,2071,1)

# Fixed mission range requirement but variable speed
mission_range = 60 * 1609.344 # 60 miles = 96560.64 m
cruise_speed = 150 * 1609.344 / 3600 # 150 miles/hour = 67.056 m/s

solution_fidelity = {'aero':1, 'hover_climb':0}

def Boltzman_Sigmoid_Battery_Pack_GED(t, scenario):

	if scenario == 'conservative':
		models = [9.6, 659.1, 0.0436, 2042.5]
		derating_factor = 0.7
	elif scenario == 'nominal':
		models = [0.0, 801.8, 0.0607, 2030.8]
		derating_factor = 0.8
	elif scenario == 'aggresive':
		models = [32.5, 1009.0, 0.0786, 2030.6]
		derating_factor = 0.9

	a = models[0]; b = models[1]; c = models[2]; tau = models[3]
	res = a + b / (1 + np.exp(c * (tau - t)))
	
	return derating_factor*res

# Battery dictionary
battery_dict = {'conservative':None, 'nominal':None, 'aggresive':None}

for i, scenario in enumerate(battery_dict):
	battery_dict[scenario] = Boltzman_Sigmoid_Battery_Pack_GED(year_list, scenario)

# --- Optimal designs by year ---
if produce_utopian_data:		

	for i, scenario in enumerate(battery_dict):

		for j, year in enumerate(year_list):

			k = 41*i+j

			battery_energy_density = battery_dict[scenario][j]

			print(f'{k}, Scenario= {scenario}; Year= {year}; Battery GED= {battery_energy_density} Wh/kg')
			sys.stdout.flush() # To flush the above print output

			if scenario == 'aggresive':
				if year in [2031,2032,2033,2035,2037,2038,2041,2045,2046,2051,2055,2056,2057,2061,2065,2068]:
					mtow_guess = 1500.0 # kg
				elif year in [2047,2048]:
					mtow_guess = 1200.0
				else:
					mtow_guess = 1000.0 # kg

			if scenario == 'nominal':
				if year in [2037,2038,2046,2050,2051,2055,2061,2062,2065,2070]:
					mtow_guess = 2000.0 # kg
				elif year in [2031,2034,2049,2058,2059,2064]:
					mtow_guess = 1500.0 # kg
				elif year in [2032]:
					mtow_guess = 1200.0 # kg
				else:
					mtow_guess = 1000.0 # kg

			if scenario == 'conservative':
				if year in [2063, 2068]:
					mtow_guess = 1500.0 # kg
				else:
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
				results = {'scenario':scenario, 'year':year, 'battery_energy_density':battery_energy_density}
				results.update(RunStandardSingleObjectiveOptimization(vehicle, mission, solution_fidelity, 'energy', mtow_guess, speed_as_design_var=False, print=True))
				results_df = pd.DataFrame(results, index=[k])
				# print(results)
				results_df.to_csv(f'optimal_results_by_scenario_by_year.csv', mode='a', header=True if k==0 else False)

if evaluate_selected_designs:

	utopian_data = pd.read_csv('optimal_results_by_scenario_by_year.csv')

	counter = -1
	for i, scenario in enumerate(battery_dict):

		opt_results = utopian_data[utopian_data['scenario']==scenario]
		battery_energy_density_list = utopian_data[utopian_data['scenario']==scenario]['battery_energy_density'].to_numpy()

		# print(len(year_list))
		for j, year1 in enumerate(year_list):

			for k, year2 in enumerate(year_list):
				
				counter += 1

				if year1 in selected_years and year1<=year2:

					battery_energy_density_opt = battery_energy_density_list[j]
					battery_energy_density_test = battery_energy_density_list[k]

					print(f'No={counter}; Scenario={scenario}; Optimized in  Year={year1}; tested in Year={year2}')
					sys.stdout.flush() # To flush the above print output

					wing_area = opt_results[opt_results['year']==year1]['Wing|area'].to_numpy()[0]
					wing_aspect_ratio = opt_results[opt_results['year']==year1]['Wing|aspect_ratio'].to_numpy()[0]
					r_lift_rotor = opt_results[opt_results['year']==year1]['LiftRotor|radius'].to_numpy()[0]
					r_propeller = opt_results[opt_results['year']==year1]['Propeller|radius'].to_numpy()[0]
					RPM_propeller_cruise = opt_results[opt_results['year']==year1]['Propeller|Cruise|RPM'].to_numpy()[0]
					cruise_speed = opt_results[opt_results['year']==year1]['cruise_speed'].to_numpy()[0]

					# Parasite drag of reference vehicle
					f_non_hub_non_wing = opt_results[opt_results['year']==year1]['Aero|Cruise|f_total_non_hub_non_wing'].to_numpy()[0]
					Cd0_wing = opt_results[opt_results['year']==year1]['Aero|Cruise|Cd0_wing'].to_numpy()[0]

					# Standard vehicle
					design_var = {'wing_area': wing_area, 'wing_aspect_ratio': wing_aspect_ratio, 'r_lift_rotor': r_lift_rotor, 'r_propeller': r_propeller}
					operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':RPM_propeller_cruise}}
					tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
					vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

					# Original battery density
					vehicle.battery.density = float(battery_energy_density_opt)

					# Fixed assumed parasite drag from the ref vehicle
					vehicle.f_total_non_hub_non_wing = {'climb': None, 'cruise': f_non_hub_non_wing, 'descent': None}
					vehicle.wing.Cd0 = {'climb': None, 'cruise': Cd0_wing, 'descent': None}

					# Standard mission
					mission = StandardMissionProfile(mission_range, cruise_speed*1000/3600)

					analysis = WeightAnalysis(vehicle, mission, solution_fidelity, weight_type='maximum', sizing_mode=True, solved_by='optimization')
					res = analysis.evaluate(record=False, weight_guess=2500.0, print=False)

					# Now swap the battery
					vehicle.battery.density = float(battery_energy_density_test)

					# GTOW Analysis
					gtow_analysis = WeightAnalysis(vehicle, mission, solution_fidelity, weight_type='gross', sizing_mode=True, solved_by='optimization')
					res = gtow_analysis.evaluate(record=False, weight_guess=2500.0, print=False)

					# Bookkeeping results
					results = {'scenario':scenario,
							   'year_opt':year1,
							   'year_test':year2,
							   'battery_energy_density_opt':battery_energy_density_opt,
							   'battery_energy_density_test':battery_energy_density_test}

					# Mission requirements
					results['mission_range'] = mission.segments[2].distance/1000.0 	# km
					results['cruise_speed'] = res.get_val('Mission|segment_3|speed', 'km/h')[0]
					results['mission_time'] = res.get_val('Mission|total_time', 'h')[0]

					# Design objective, variables, and constraints
					results['Weight|max_takeoff'] = vehicle.weight.max_takeoff
					results['Weight|gross_takeoff'] = vehicle.weight.gross_takeoff
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
					results_df = pd.DataFrame(results, index=[counter])
					results_df.to_csv(f'opt_test_results.csv', mode='a', header=True if counter==0 else False)

if plot_utopian_data:

	parameter = 'Energy|entire_mission'
	# parameter = 'cruise_speed'
	# parameter = 'Weight|takeoff'
	# parameter = 'Wing|area'
	# parameter = 'Wing|aspect_ratio'
	# parameter = 'LiftRotor|radius'
	# parameter = 'Propeller|radius'
	# parameter = 'Propeller|Cruise|RPM'

	utopian_data = pd.read_csv('optimal_results_by_scenario_by_year.csv')

	fig, axes = plt.subplots(1, 3, figsize=(14, 6), sharey=True)

	for i, scenario in enumerate(battery_dict):

		data = utopian_data[utopian_data['scenario']==scenario]

		# Plot data on each subplot
		axes[i].plot(data['year'], data[parameter], 'k--', markersize=4)
		axes[i].set_title(f'{scenario}')
		axes[i].set_xlabel('Year')
		if i==0: axes[i].set_ylabel(f'{parameter}')

	plt.subplots_adjust(bottom=0.2, top=0.82, wspace=0.1)
	fig.suptitle(f'{parameter} by year based on different battery projection scenarios', size=16)
	plt.show()

if plot_evaluation_data:

	parameter = 'Energy|entire_mission'
	# parameter = 'cruise_speed'
	# parameter = 'Weight|takeoff'
	# parameter = 'Wing|area'
	# parameter = 'Wing|aspect_ratio'
	# parameter = 'LiftRotor|radius'
	# parameter = 'Propeller|radius'
	# parameter = 'Propeller|Cruise|RPM'

	utopian_data = pd.read_csv('optimal_results_by_scenario_by_year.csv')
	opt_test_data = pd.read_csv('opt_test_results.csv')

	fig, axes = plt.subplots(1, 3, figsize=(14, 6), sharey=True)

	for i, scenario in enumerate(battery_dict):

		data = utopian_data[utopian_data['scenario']==scenario]	
		test_data = opt_test_data[opt_test_data['scenario']==scenario]

		# Plot data on each subplot
		axes[i].plot(data['year'], data[parameter], 'k--', markersize=4, label='Utopian energy' if i==0 else None)
		axes[i].set_title(f'{scenario}')
		axes[i].set_xlabel('Year')
		if i==0: axes[i].set_ylabel(r'Energy consumption $[kWh]$')

		# Plot evaluation data
		for year_opt in selected_years:
			test_data2 = test_data[test_data['year_opt']==year_opt]
			axes[i].plot(test_data2['year_test'], test_data2[parameter], 'r-' if year_opt==2030 else 'b-', label=f'Opt in {year_opt}' if i==0 else None)

	plt.subplots_adjust(bottom=0.2, top=0.82, wspace=0.1)
	fig.legend(ncols=3,bbox_to_anchor=(0.49,0.93),loc='upper center')
	fig.suptitle('Optimal energy by year based on different battery projection scenarios', size=16)
	plt.show()




