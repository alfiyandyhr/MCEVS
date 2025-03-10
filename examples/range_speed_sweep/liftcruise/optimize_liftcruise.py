from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardOptimization
from MCEVS.Applications.OpenVSP.Standard_Vehicles import create_NASA_LiftPlusCruise_vsp3
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

print(range_array)
print(speed_array)

solution_fidelity = {'aero':1, 'hover_climb':0}

if run_without_speed_as_design_var_one_opt or run_with_speed_as_design_var_one_opt:
	
	range_ij = 100

	if run_without_speed_as_design_var_one_opt: mtow_guess = 3000.0; speed_ij = 240
	if run_with_speed_as_design_var_one_opt: mtow_guess = 4000.0; speed_ij = 300

	# Standard vehicle
	design_var = {'wing_area': 30.0, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
	operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':380.0}}
	tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
	vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

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
	wing_area_guess = 30.0
	for i, mission_range in enumerate(range_array):
		for j, cruise_speed in enumerate(speed_array):

			iter_idx = i*len(speed_array)+j+1
			print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}")
			sys.stdout.flush() # To flush the above print output

			# Standard vehicle
			design_var = {'wing_area': wing_area_guess, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
			operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':380.0}}
			tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
			vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

			# Different initial guesses
			if mission_range in [10] and cruise_speed == 120: mtow_guess = 1000.0
			if mission_range in [20] and cruise_speed == 120: mtow_guess = 1500.0
			if mission_range in [110] and cruise_speed == 140: mtow_guess = 2500.0
			if mission_range in [110] and cruise_speed == 150: mtow_guess = 2500.0
			if mission_range in [60,70,90] and cruise_speed == 160: mtow_guess = 2500.0
			if mission_range in [160] and cruise_speed == 160: mtow_guess = 2000.0
			if mission_range in [90,130] and cruise_speed == 170: mtow_guess = 2500.0
			if mission_range in [70,100] and cruise_speed == 180: mtow_guess = 2000.0
			if mission_range == 220 and cruise_speed in [190,230]: mtow_guess = 5000.0
			if mission_range in [190,200] and cruise_speed == 250: mtow_guess = 5000.0
			if mission_range in [20] and cruise_speed == 260: mtow_guess = 2500.0
			if mission_range in [220] and cruise_speed == 260: mtow_guess = 5000.0
			if mission_range in [210] and cruise_speed == 270: mtow_guess = 5000.0
			if mission_range in [150] and cruise_speed == 280: mtow_guess = 2500.0
			if mission_range in [170] and cruise_speed == 280: mtow_guess = 5000.0
			if mission_range in [110] and cruise_speed == 320: mtow_guess = 5000.0

			# Standard mission
			mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed*1000/3600)

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
	wing_area_guess = 30.0
	for i, mission_range in enumerate(range_array):
		for j, cruise_speed in enumerate(speed_array):

			if mission_range in [210] and cruise_speed == 170:
				iter_idx = i*len(speed_array)+j+1
				print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}")
				sys.stdout.flush() # To flush the above print output

				# Standard vehicle
				design_var = {'wing_area': wing_area_guess, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
				operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':380.0}}
				tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
				vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

				# Different initial guesses
				if mission_range in [10] and cruise_speed == 120: mtow_guess = 1000.0
				if mission_range in [20] and cruise_speed == 120: mtow_guess = 1500.0
				if mission_range in [110] and cruise_speed == 140: mtow_guess = 2500.0
				if mission_range in [110] and cruise_speed == 150: mtow_guess = 2500.0
				if mission_range in [60,70,90] and cruise_speed == 160: mtow_guess = 2500.0
				if mission_range in [160] and cruise_speed == 160: mtow_guess = 2000.0
				if mission_range in [90,130] and cruise_speed == 170: mtow_guess = 2500.0
				if mission_range in [70,100] and cruise_speed == 180: mtow_guess = 2000.0
				if mission_range == 220 and cruise_speed in [190,230]: mtow_guess = 5000.0
				if mission_range in [190,200] and cruise_speed == 250: mtow_guess = 5000.0
				if mission_range in [20] and cruise_speed == 260: mtow_guess = 2500.0
				if mission_range in [220] and cruise_speed == 260: mtow_guess = 5000.0
				if mission_range in [210] and cruise_speed == 270: mtow_guess = 5000.0
				if mission_range in [150] and cruise_speed == 280: mtow_guess = 2500.0
				if mission_range in [170] and cruise_speed == 280: mtow_guess = 5000.0
				if mission_range in [110] and cruise_speed == 320: mtow_guess = 5000.0

				# Standard mission
				mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed*1000/3600)

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

		cruise_speed_guess = 300 # km/h
		if mission_range in [100,190]: mtow_guess = 4000.0
		elif mission_range == 10: mtow_guess = 1000.0
		elif mission_range == 20: mtow_guess = 1500.0
		elif mission_range == 220: mtow_guess = 3000.0
		else: mtow_guess = 2000.0

		# Standard vehicle
		design_var = {'wing_area': 30.0, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
		operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':380.0}}
		tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
		vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

		print(f"Iter= {i}, Range= {mission_range}, Speed guess= {cruise_speed_guess}")

		# Standard mission
		mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed_guess*1000/3600)

		# Standard optimization
		results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=True, print=False)
		# print(results)

		results_df = pd.DataFrame(results, index=[i])

		results_df.to_csv('results_with_speed_as_design_var.csv', mode='a', header=True if i==0 else False)

