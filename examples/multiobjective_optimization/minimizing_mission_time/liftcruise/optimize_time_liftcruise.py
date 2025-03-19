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
	speed_ij = 300 # km/h

	mtow_guess = 1000.0

	# Standard vehicle
	design_var = {'wing_area': 30.0, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
	operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':380.0}}
	tfs = {'tf_structure':0.8, 'tf_propulsion':0.8, 'tf_equipment':0.8}
	vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6, payload_per_pax=100.0)

	# Changed battery density
	vehicle.battery.density = float(battery_energy_density)

	# Standard mission
	mission_ij = StandardMissionProfile(range_ij*1000, speed_ij*1000/3600)

	# Standard optimization
	with warnings.catch_warnings():
		warnings.simplefilter("ignore")
		results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, solution_fidelity, 'mission_time', mtow_guess, speed_as_design_var=True, print=True)
		print(results)

# Expensive simulations, run once !!!
if run_with_speed_as_design_var_all_opt:

	for i, mission_range in enumerate(range_array):

		sys.stdout.flush() # To flush the above print output

		if battery_energy_density == 250:
			cruise_speed_guess = 300 # km/h
			if mission_range in [20,40,50,60]: mtow_guess = 2000.0
			elif mission_range in [120,130,140,150]: mtow_guess = 3000.0
			elif mission_range in [160]: mtow_guess = 2500.0
			elif mission_range in [200]: mtow_guess = 9000.0; cruise_speed_guess = 290
			elif mission_range in [210]: mtow_guess = 1070.0; cruise_speed_guess = 320
			elif mission_range in [220]: mtow_guess = 9999.0
			elif mission_range in [180]: mtow_guess = 5000.0
			elif mission_range in [190]: mtow_guess = 6000.0
			elif mission_range in [170]: mtow_guess = 3000.0
			else: mtow_guess = 1000.0
		elif battery_energy_density == 400:
			cruise_speed_guess = 300 # km/h
			mtow_guess = 1000.0
		elif battery_energy_density == 550:
			cruise_speed_guess = 300 # km/h
			mtow_guess = 1000.0

		# Standard vehicle
		design_var = {'wing_area': 30.0, 'wing_aspect_ratio': 12.0, 'r_lift_rotor': 1.524, 'r_propeller': 1.37}
		operation_var = {'RPM_lift_rotor':{'hover_climb':400.0}, 'RPM_propeller': {'cruise':380.0}}
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
			results = RunStandardSingleObjectiveOptimization(vehicle, mission_ij, solution_fidelity, 'mission_time', mtow_guess, speed_as_design_var=True, print=True)
			# print(results)

		results_df = pd.DataFrame(results, index=[i])

		results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv', mode='a', header=True if i==0 else False)
		# results_df.to_csv(f'battery_{battery_energy_density}_Whpkg/results_test.csv', mode='a', header=True if i==0 else False)
