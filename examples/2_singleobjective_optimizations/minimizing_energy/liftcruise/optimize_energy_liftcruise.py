from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardSingleObjectiveOptimization
from MCEVS.Applications.OpenVSP.Standard_Vehicles import create_NASA_LiftPlusCruise_vsp3
import numpy as np
import pandas as pd
import sys
import warnings

run_with_speed_as_design_var_one_opt = True
run_with_speed_as_design_var_all_opt = False

battery_energy_density = 250 # [250,400,550]

# Mission requirement sweep
range_i, range_f, d_range = 10, 230, 10 # km
range_array = np.arange(range_i,range_f,d_range)

print(range_array)

solution_fidelity = {'aero':1, 'hover_climb':0}

if run_with_speed_as_design_var_one_opt:
	
	range_ij = 100 # km
	speed_ij = 150 * 1.609344 # 150 miles/hour = 241.402 km/h

	mtow_guess = 1000.0

	# Standard vehicle
	design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
	operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':500.0}}
	tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
	vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

	# Changed battery density
	vehicle.battery.density = float(battery_energy_density)

	# Standard mission
	mission_ij = StandardMissionProfile(range_ij*1000, speed_ij*1000/3600)

	# Standard optimization
	with warnings.catch_warnings():
		warnings.simplefilter("ignore")
		results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, solution_fidelity, 'energy', mtow_guess, speed_as_design_var=True, print=True)
		print(results)

# Expensive simulations, run once !!!
if run_with_speed_as_design_var_all_opt:

	for i, mission_range in enumerate(range_array):

		sys.stdout.flush() # To flush the above print output

		cruise_speed_guess = 150 * 1.609344 # 150 miles/hour = 241.402 km/h

		if battery_energy_density == 250:
			if mission_range in [10,30]: mtow_guess = 1200.0
			elif mission_range in [210]: mtow_guess = 1500.0
			else: mtow_guess = 1000.0
		elif battery_energy_density == 400:
			if mission_range in [20]: mtow_guess = 950.0
			elif mission_range in [50]: mtow_guess = 1250.0
			elif mission_range in [70]: mtow_guess = 1500.0
			elif mission_range in [80]: mtow_guess = 1300.0
			elif mission_range in [90]: mtow_guess = 1210.0
			elif mission_range in [40,100]: mtow_guess = 1100.0
			elif mission_range in [60,110]: mtow_guess = 1200.0
			else: mtow_guess = 1000.0
		elif battery_energy_density == 550:
			if mission_range in [20]: mtow_guess = 1150.0
			elif mission_range in [40]: mtow_guess = 950.0
			elif mission_range in [50]: mtow_guess = 860.0
			elif mission_range in [60]: mtow_guess = 905.0
			elif mission_range in [30,70]: mtow_guess = 1300.0
			elif mission_range in [80]: mtow_guess = 1700.0
			elif mission_range in [90]: mtow_guess = 1280.0
			elif mission_range in [100]: mtow_guess = 1350.0
			elif mission_range in [10,110,120]: mtow_guess = 1500.0
			else: mtow_guess = 1000.0

		# Standard vehicle
		design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
		operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':500.0}}
		tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
		vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

		# Changed battery density
		vehicle.battery.density = float(battery_energy_density)

		print(f"Iter= {i}, Range= {mission_range}, Speed guess= {cruise_speed_guess}")

		# Standard mission
		mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed_guess*1000/3600)

		# Standard optimization
		with warnings.catch_warnings():
			warnings.simplefilter("ignore")
			results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, solution_fidelity, 'energy', mtow_guess, speed_as_design_var=True, print=False)
			# print(results)

		results_df = pd.DataFrame(results, index=[i])

		results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv', mode='a', header=True if i==0 else False)
		# results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_test.csv', mode='a', header=True if i==0 else False)
