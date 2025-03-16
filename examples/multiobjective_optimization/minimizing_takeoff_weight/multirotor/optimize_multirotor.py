from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardSingleObjectiveOptimization
import numpy as np
import pandas as pd
import sys
import time

run_with_speed_as_design_var_one_opt = True
run_with_speed_as_design_var_all_opt = False

objective = 'takeoff_weight'
# objective = 'energy'
# objective = 'mission_time'

battery_energy_density = 250 # [250,400,550]

# Mission requirement sweep
range_i, range_f, d_range = 10, 230, 10 # km
range_array = np.arange(range_i,range_f,d_range)

print(range_array, len(range_array))

solution_fidelity = {'aero':1, 'hover_climb':0}

if run_with_speed_as_design_var_one_opt:

	range_ij = 100 # km
	speed_ij = 300 # km/h

	mtow_guess = 1000.0

	# Standard vehicle
	design_var = {'r_lift_rotor': 4.0}
	operation_var = {'RPM_lift_rotor': {'hover_climb':400.0, 'cruise':450.0}}
	tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
	vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

	# Changed battery density
	vehicle.battery.density = float(battery_energy_density)

	# Standard mission
	mission_ij = StandardMissionProfile(range_ij*1000, speed_ij*1000/3600)

	# Standard optimization
	if objective == 'takeoff_weight':
		results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, solution_fidelity, 'takeoff_weight', mtow_guess, speed_as_design_var=True, print=True)
	elif objective == 'energy':
		results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, solution_fidelity, 'energy', mtow_guess, speed_as_design_var=True, print=True)
	elif objective == 'mission_time':
		results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, solution_fidelity, 'mission_time', mtow_guess, speed_as_design_var=True, print=True)
	print(results)

# Expensive simulations, run once !!!
if run_with_speed_as_design_var_all_opt:

	for i, mission_range in enumerate(range_array):

		sys.stdout.flush() # To flush the above print output

		if battery_energy_density == 250:
			if mission_range in [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160]:
				cruise_speed_guess = 300 # km/h
				mtow_guess = 1000.0
			if mission_range in [170, 180, 190, 200, 210, 220]:
				cruise_speed_guess = 300 # km/h
				mtow_guess = 5000.0
		elif battery_energy_density == 400:
			cruise_speed_guess = 300 # km/h
			mtow_guess = 1000.0
		elif battery_energy_density == 550:
			cruise_speed_guess = 300 # km/h
			if mission_range in [30]: mtow_guess = 1500.0
			else: mtow_guess = 1000.0

		# Standard vehicle
		design_var = {'r_lift_rotor': 4.0}
		operation_var = {'RPM_lift_rotor': {'hover_climb':400.0, 'cruise':450.0}}
		tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
		vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

		# Changed battery density
		vehicle.battery.density = float(battery_energy_density)

		print(f"Iter= {i}, Range= {mission_range}, Speed guess= {cruise_speed_guess}")

		# Standard mission
		mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed_guess*1000/3600)

		# Standard optimization
		results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=True, print=False)
		# print(results)

		results_df = pd.DataFrame(results, index=[i])

		results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv', mode='a', header=True if i==0 else False)


