from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Optimization.Standard import RunStandardOptimization
from MCEVS.Applications.OpenVSP.Standard_Vehicles import create_NASA_LiftPlusCruise_vsp3
import numpy as np
import pandas as pd
import sys

run_without_speed_as_design_var_one_opt = True
run_with_speed_as_design_var_one_opt = False
run_without_speed_as_design_var_all_opt = False
run_with_speed_as_design_var_all_opt = False

# Mission requirement sweep
range_i, range_f, d_range = 10, 160, 10 	# km
speed_i, speed_f, d_speed = 36, 600, 36 	# km/h
# range_i, range_f, d_range = 10000, 20000, 10000 	# m
# speed_i, speed_f, d_speed = 20, 30, 10 			# m/s
range_array = np.arange(range_i,range_f,d_range)
speed_array = np.arange(speed_i,speed_f,d_speed)

# print(range_array)
# print(speed_array)

# Init value guess
init_value_guess = {'Weight|takeoff': 1500.0,
					'LiftRotor|radius': 4.0,
					'LiftRotor|Cruise|RPM': 400.0}

solution_fidelity = {'aero':1, 'hover_climb':0}

if run_without_speed_as_design_var_one_opt or run_with_speed_as_design_var_one_opt:
	mtow_guess = 2000.0

	# Standard vehicle
	design_var = {'wing_area': 20.0, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
	operation_var = {'RPM': {'cruise':400.0}}
	tfs = {'tf_structure':0.8, 'tf_propulsion':1.0, 'tf_equipment':0.8}
	vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

	# vehicle.print_info()

	# b = np.sqrt(design_var['wing_area'] * design_var['wing_aspect_ratio'])
	# print(vehicle.fuselage.max_diameter)

	# print(3*2*design_var['r_lift_rotor'] + vehicle.fuselage.max_diameter - 0.95*b)
	# print(3*2*2.499999989678195 + vehicle.fuselage.max_diameter - 0.95*np.sqrt(21.887595113256005*6.000000253385185))


	# create_NASA_LiftPlusCruise_vsp3('vehicle.vsp3', vehicle)

	# Standard mission
	mission_ij = StandardMissionProfile(160*1000, 252*1000/3600)

	# Standard optimization
	if run_without_speed_as_design_var_one_opt:
		results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=False, print=True)
		print(results)
	if run_with_speed_as_design_var_one_opt:
		results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=True, print=True)
		print(results)

# Expensive simulations, run once !!!
if run_without_speed_as_design_var_all_opt:
	mtow_guess = 2000
	for i, mission_range in enumerate(range_array):
		for j, cruise_speed in enumerate(speed_array):

			if cruise_speed == 252:
				iter_idx = i*len(speed_array)+j+1
				print(f"Iter= {iter_idx}, Range= {mission_range}, Speed= {cruise_speed}")
				sys.stdout.flush() # To flush the above print output

				# Standard vehicle
				design_var = {'wing_area': 20.0, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
				operation_var = {'RPM': {'cruise':400.0}}
				tfs = {'tf_structure':1.0, 'tf_propulsion':1.0, 'tf_equipment':1.0}
				vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

				# Standard mission
				mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed*1000/3600)

				# # Different initial guesses
				# if mission_range in [40.0, 30.0] and cruise_speed == 108.0: mtow_guess = 1300.0
				# if mission_range in [50.0, 30.0, 20.0] and cruise_speed == 144.0: mtow_guess = 1400.0
				# if mission_range in [110.0] and cruise_speed == 180.0: mtow_guess = 1700.0
				# if mission_range in [90.0] and cruise_speed == 180.0: mtow_guess = 1600.0
				# if mission_range in [80.0] and cruise_speed == 180.0: mtow_guess = 1410.0
				# if mission_range in [70.0] and cruise_speed == 180.0: mtow_guess = 1400.0
				# if mission_range in [30.0, 40.0] and cruise_speed == 180.0: mtow_guess = 1200.0
				# if mission_range == 10.0 and cruise_speed in [180.0, 252.0]: mtow_guess = 1000.0
				# if mission_range in [60.0] and cruise_speed == 216.0: mtow_guess = 1240.0
				# if mission_range in [130.0] and cruise_speed == 216.0: mtow_guess = 1700.0
				# if mission_range in [50.0] and cruise_speed == 252.0: mtow_guess = 1250.0
				# if mission_range in [20.0] and cruise_speed == 324.0: mtow_guess = 1200.0
				# if mission_range in [40.0] and cruise_speed == 360.0: mtow_guess = 1400.0
				# if mission_range in [10.0] and cruise_speed == 360.0: mtow_guess = 1010.0
				# if mission_range in [50.0] and cruise_speed == 396.0: mtow_guess = 1600.0
				# if mission_range in [30.0] and cruise_speed == 396.0: mtow_guess = 1300.0
				# if mission_range in [20.0] and cruise_speed == 396.0: mtow_guess = 1100.0
				# if mission_range in [10.0] and cruise_speed == 432.0: mtow_guess = 1155.0
				# if mission_range in [20.0] and cruise_speed == 432.0: mtow_guess = 1150.0
				# if mission_range in [30.0] and cruise_speed == 432.0: mtow_guess = 1590.0
				# if mission_range in [40.0, 50.0] and cruise_speed == 432.0: mtow_guess = 1600.0
				# if mission_range in [10.0] and cruise_speed == 468.0: mtow_guess = 1080.0
				# if mission_range in [20.0] and cruise_speed == 468.0: mtow_guess = 1100.0
				# if mission_range in [40.0] and cruise_speed == 468.0: mtow_guess = 1600.0
				# if mission_range == 10.0 and cruise_speed in [504.0, 540.0, 576.0]: mtow_guess = 400.0
				# if mission_range in [20.0] and cruise_speed == 504.0: mtow_guess = 1090.0
				# if mission_range == 20.0 and cruise_speed in [540.0, 576.0] : mtow_guess = 1000.0

				# Standard optimization
				results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=False, print=False)
				print(results)

				results_df = pd.DataFrame(results, index=[iter_idx])

				# results_df.to_csv('Liftcruise_OPT_results_without_speed_as_design_var.csv', mode='a', header=True if iter_idx==1 else False)
				results_df.to_csv('Liftcruise_OPT_results_test.csv', mode='a', header=True if iter_idx==7 else False)

				# Change back to the default guess
				mtow_guess = 2000.0

# Expensive simulations, run once !!!
if run_with_speed_as_design_var_all_opt:
	mtow_guess = 1500
	cruise_speed_guess = 216.0 # km/h # best speed for all range !!
	for i, mission_range in enumerate(range_array):
		sys.stdout.flush() # To flush the above print output

		if mission_range == 40.0: r_lift_rotor_guess = 2.846634; cruise_RPM_guess = 401.058084
		else: r_lift_rotor_guess = 4.0; cruise_RPM_guess = 400.0

		if mission_range == 20.0: mtow_guess = 500.0;

		# Standard vehicle
		design_var = {'r_lift_rotor': r_lift_rotor_guess}
		operation_var = {'RPM': {'cruise':cruise_RPM_guess}}
		tfs = {'tf_structure':1.0, 'tf_propulsion':1.0, 'tf_equipment':1.0}
		vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

		print(f"Iter= {i}, Range= {mission_range}, Speed guess= {cruise_speed_guess}")

		# Standard mission
		mission_ij = StandardMissionProfile(mission_range*1000, cruise_speed_guess*1000/3600)

		# Standard optimization
		results = RunStandardOptimization(vehicle, mission_ij, solution_fidelity, mtow_guess, speed_as_design_var=True, print=False)
		# print(results)

		results_df = pd.DataFrame(results, index=[i])

		results_df.to_csv('Liftcruise_OPT_results_with_speed_as_design_var.csv', mode='a', header=True if i==0 else False)

		# Change back to the default guess
		mtow_guess = 1500.0


