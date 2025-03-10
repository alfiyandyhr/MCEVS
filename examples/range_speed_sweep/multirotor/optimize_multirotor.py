from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardOptimization
import numpy as np
import pandas as pd
import sys
import time

run_without_speed_as_design_var_one_opt = True
run_with_speed_as_design_var_one_opt = False

run_without_speed_as_design_var_all_opt = False
run_with_speed_as_design_var_all_opt = False

rerun_without_speed_as_design_var_all_opt = False

# Mission requirement sweep
range_i, range_f, d_range = 10, 230, 10 	# km
speed_i, speed_f, d_speed = 80, 330, 10 	# km/h
range_array = np.arange(range_i,range_f,d_range)
speed_array = np.arange(speed_i,speed_f,d_speed)

print(range_array, len(range_array))
print(speed_array, len(speed_array))

solution_fidelity = {'aero':1, 'hover_climb':0}

if run_without_speed_as_design_var_one_opt or run_with_speed_as_design_var_one_opt:

	range_ij = 100 # km

	if run_without_speed_as_design_var_one_opt: mtow_guess = 3000.0; speed_ij = 240
	if run_with_speed_as_design_var_one_opt: mtow_guess = 1000.0; speed_ij = 300

	# Standard vehicle
	design_var = {'r_lift_rotor': 4.0}
	operation_var = {'RPM_lift_rotor': {'hover_climb':400.0, 'cruise':450.0}}
	tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
	vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

	# Standard mission
	mission_ij = StandardMissionProfile(range_ij*1000, speed_ij*1000/3600)

	# Standard optimization
	if run_without_speed_as_design_var_one_opt:
		results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=False, print=True)
		print(results)
	if run_with_speed_as_design_var_one_opt:
		results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=True, print=True)
		print(results)

# Expensive simulations, run once !!!
if run_without_speed_as_design_var_all_opt:
	t1 = time.time()
	mtow_guess = 3000.0
	for i, mission_range in enumerate(range_array):
		for j, cruise_speed in enumerate(speed_array):

			# if mission_range in [220] and cruise_speed == 240:
			iter_idx = i*len(speed_array)+j+1
			print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}")
			sys.stdout.flush() # To flush the above print output

			# Standard vehicle
			design_var = {'r_lift_rotor': 4.0}
			operation_var = {'RPM_lift_rotor': {'hover_climb':400.0, 'cruise':450.0}}
			tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
			vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

			# Standard mission
			mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed*1000/3600)

			# Different initial guesses
			if mission_range in [30] and cruise_speed == 90: mtow_guess = 2500.0
			if mission_range in [50] and cruise_speed == 90: mtow_guess = 3500.0
			if mission_range in [50] and cruise_speed == 100: mtow_guess = 2500.0
			if mission_range in [60] and cruise_speed == 110: mtow_guess = 3500.0
			if mission_range in [40] and cruise_speed == 160: mtow_guess = 2500.0
			if mission_range in [80] and cruise_speed == 170: mtow_guess = 2500.0
			if mission_range in [90] and cruise_speed == 190: mtow_guess = 2500.0
			if mission_range in [160] and cruise_speed == 280: mtow_guess = 5000.0

			# Standard optimization
			results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=False, print=False)
			# print(results)

			results_df = pd.DataFrame(results, index=[iter_idx])

			results_df.to_csv('results_without_speed_as_design_var.csv', mode='a', header=True if iter_idx==1 else False)

			# Change back to the default guess
			mtow_guess = 3000.0

	t2 = time.time()
	np.savetxt('computational_time.dat', np.array([t2-t1]))
	print(t2-t1)

# Expensive simulations, run once !!!
if rerun_without_speed_as_design_var_all_opt:
	mtow_guess = 3000.0
	for i, mission_range in enumerate(range_array):
		for j, cruise_speed in enumerate(speed_array):

			if mission_range in [100] and cruise_speed == 200:
				iter_idx = i*len(speed_array)+j+1
				print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}")
				sys.stdout.flush() # To flush the above print output

				# Standard vehicle
				design_var = {'r_lift_rotor': 4.0}
				operation_var = {'RPM_lift_rotor': {'hover_climb':400.0, 'cruise':450.0}}
				tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
				vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

				# Standard mission
				mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed*1000/3600)

				# Different initial guesses
				if mission_range in [30] and cruise_speed == 90: mtow_guess = 2500.0
				if mission_range in [50] and cruise_speed == 90: mtow_guess = 3500.0
				if mission_range in [50] and cruise_speed == 100: mtow_guess = 2500.0
				if mission_range in [60] and cruise_speed == 110: mtow_guess = 3500.0
				if mission_range in [40] and cruise_speed == 160: mtow_guess = 2500.0
				if mission_range in [80] and cruise_speed == 170: mtow_guess = 2500.0
				if mission_range in [90] and cruise_speed == 190: mtow_guess = 2500.0
				if mission_range in [160] and cruise_speed == 280: mtow_guess = 5000.0

				# Standard optimization
				results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=False, print=False)
				# print(results)

				results['Unnamed: 0'] = iter_idx
				results_df = pd.DataFrame(results, index=[iter_idx])
				last_column = results_df.columns[-1]
				results_df = results_df[[last_column] + [col for col in results_df if col != last_column]]

				results_df_ori = pd.read_csv('results_without_speed_as_design_var.csv')
				results_df_ori.iloc[iter_idx-1] = results_df.iloc[0]

				results_df_ori.to_csv('results_without_speed_as_design_var.csv', index=False)

				# Change back to the default guess
				mtow_guess = 3000.0

# Expensive simulations, run once !!!
if run_with_speed_as_design_var_all_opt:

	for i, mission_range in enumerate(range_array):

		sys.stdout.flush() # To flush the above print output

		if mission_range in [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160]:
			cruise_speed_guess = 300 # km/h
			mtow_guess = 1000.0
		if mission_range in [170, 180, 190, 200, 210, 220]:
			cruise_speed_guess = 300 # km/h
			mtow_guess = 5000.0

		# Standard vehicle
		design_var = {'r_lift_rotor': 4.0}
		operation_var = {'RPM_lift_rotor': {'hover_climb':400.0, 'cruise':450.0}}
		tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
		vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

		print(f"Iter= {i}, Range= {mission_range}, Speed guess= {cruise_speed_guess}")

		# Standard mission
		mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed_guess*1000/3600)

		# Standard optimization
		results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=True, print=False)
		# print(results)

		results_df = pd.DataFrame(results, index=[i])

		results_df.to_csv('results_with_speed_as_design_var.csv', mode='a', header=True if i==0 else False)


