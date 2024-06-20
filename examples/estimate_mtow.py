import numpy as np
import openmdao.api as om
import matplotlib.pyplot as plt

from MCEVS.Weights.Groups import MTOWEstimation

if __name__ == '__main__':

	# --- eVTOL 1: Wingless Multirotor --- #
	# General parameters
	evtol1_params = {}
	evtol1_params['evtol_config'] 			= 'multirotor'
	evtol1_params['N_rotors_lift'] 			= 7
	evtol1_params['rotor_lift_solidity'] 	= 0.13
	evtol1_params['hover_FM'] 				= 0.75
	# Fuselage parameters
	evtol1_params['n_pax']					= 4 # number of passengers (including pilot)
	evtol1_params['l_fuse']					= 6.7 # m
	evtol1_params['d_fuse_max']				= 1.8 # m
	# Landing gear parameters
	evtol1_params['l_sm']					= 0.7 # m
	evtol1_params['n_ult_lg']				= 5.7
	# Battery parameters
	evtol1_params['battery_rho'] 			= 250.0 # Wh/kg
	evtol1_params['battery_eff'] 			= 0.85
	evtol1_params['battery_max_discharge'] 	= 0.8
	# Design parameters
	evtol1_r_rotor_lift		= 1.2 # m
	evtol1_cruise_speed 	= 50.0 # m/s
	evtol1_rotor_mu 		= 0.3

	# Constants
	evtol1_params['rho_air'] 				= 1.225 # kg/m**3
	evtol1_params['gravitational_accel'] 	= 9.81 # kg/m**3


	# --- eVTOL 2: Lift+Cruise --- #
	# General parameters
	evtol2_params = {}
	evtol2_params['evtol_config'] 			= 'lift+cruise'
	evtol2_params['N_rotors_lift'] 			= 6
	evtol2_params['N_rotors_cruise'] 		= 1
	evtol2_params['rotor_lift_solidity'] 	= 0.13
	evtol2_params['rotor_cruise_solidity']	= 0.13
	evtol2_params['hover_FM'] 				= 0.75
	# Wing parameters
	# evtol2_params['Cd0'] = 0.0397
	evtol2_params['wing_AR'] 				= 10.0
	evtol2_params['n_ult_wing']				= 3.0
	# Fuselage parameters
	evtol2_params['n_pax']					= 4 # number of passengers (including pilot)
	evtol2_params['l_fuse']					= 6.7 # m
	evtol2_params['d_fuse_max']				= 1.8 # m
	# Landing gear parameters
	evtol2_params['l_sm']					= 0.7 # m
	evtol2_params['n_ult_lg']				= 5.7
	# Battery parameters
	evtol2_params['battery_rho'] 			= 250.0 # Wh/kg
	evtol2_params['battery_eff'] 			= 0.85
	evtol2_params['battery_max_discharge'] 	= 0.8
	# Design parameters
	evtol2_r_rotor_lift		= 1.2 # m
	evtol2_r_rotor_cruise 	= 0.9 # m
	evtol2_cruise_speed 	= 50.0 # m/s
	evtol2_wing_area 		= 6.4 # m**2
	evtol2_rotor_J 			= 1.0

	# Constants
	evtol2_params['AoA_cruise']				= 5.0 # deg
	evtol2_params['rho_air'] 				= 1.225 # kg/m**3
	evtol2_params['gravitational_accel'] 	= 9.81 # kg/m**3

	# --- Mission requirements --- #
	n_missions		= 20
	payload_weight	= 400.0 # kg
	flight_ranges 	= np.linspace(1000, 50000, n_missions) # m
	hover_times 	= n_missions * [240.0] # s

	# --- MTOW Estimation for Wingless Multirotor --- #
	mtow_list1 = np.zeros(n_missions)

	for i in range(n_missions):
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
		indeps.add_output('payload_weight', payload_weight, units='kg')
		indeps.add_output('flight_distance', flight_ranges[i], units='m')
		indeps.add_output('hover_time', hover_times[i], units='s')
		indeps.add_output('eVTOL|Cruise_speed', evtol1_cruise_speed, units='m/s')
		indeps.add_output('Rotor|radius_lift', evtol1_r_rotor_lift, units='m')
		indeps.add_output('Rotor|mu', evtol1_rotor_mu)
		
		prob.model.add_subsystem('mtow_estimation',
								  MTOWEstimation(evtol_options=evtol1_params, use_solver=True),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		prob.setup(check=False)
		prob.run_model()
		# prob.check_partials(compact_print=True, show_only_incorrect=True)
		mtow_list1[i] = prob.get_val('eVTOL|W_takeoff')[0]

	# --- MTOW Estimation for Lift+Cruise --- #
	mtow_list2 = np.zeros(n_missions)

	for i in range(n_missions):
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
		indeps.add_output('payload_weight', payload_weight, units='kg')
		indeps.add_output('flight_distance', flight_ranges[i], units='m')
		indeps.add_output('hover_time', hover_times[i], units='s')
		indeps.add_output('eVTOL|Cruise_speed', evtol2_cruise_speed, units='m/s')
		indeps.add_output('Rotor|radius_lift', evtol2_r_rotor_lift, units='m')
		indeps.add_output('Rotor|radius_cruise', evtol2_r_rotor_cruise, units='m')
		indeps.add_output('eVTOL|S_wing', evtol2_wing_area, units='m**2')
		indeps.add_output('Rotor|J', evtol2_rotor_J)
		
		prob.model.add_subsystem('mtow_estimation',
								  MTOWEstimation(evtol_options=evtol2_params, use_solver=True),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		prob.setup(check=False)
		prob.run_model()
		# prob.check_partials(compact_print=True, show_only_incorrect=True)
		mtow_list2[i] = prob.get_val('eVTOL|W_takeoff')[0]

	plt.plot(flight_ranges/1000.0, mtow_list1, label='Wingless Multirotor')
	plt.plot(flight_ranges/1000.0, mtow_list2, label='Winged Lift+Cruise')
	plt.xlabel('Mission range [km]')
	plt.ylabel('Maximum Takeoff Weight [kg]')
	plt.title('MTOW vs mission range')
	plt.legend(loc='upper left')
	plt.grid(True)
	plt.show()
















		