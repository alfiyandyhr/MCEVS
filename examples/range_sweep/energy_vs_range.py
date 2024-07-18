import numpy as np
import openmdao.api as om
import matplotlib.pyplot as plt

from MCEVS.Energy.Energy import EnergyConsumption

if __name__ == '__main__':

	# --- eVTOL 1: Wingless Multirotor --- #
	# General parameters
	evtol1_params = {}
	evtol1_params['evtol_config'] 			= 'multirotor'
	evtol1_params['N_rotors_lift'] 			= 7
	evtol1_params['rotor_lift_solidity'] 	= 0.13
	evtol1_params['hover_FM'] 				= 0.75
	# Battery parameters
	evtol1_params['battery_rho'] 			= 250.0 # Wh/kg
	evtol1_params['battery_eff'] 			= 0.85
	evtol1_params['battery_max_discharge'] 	= 0.8
	# Design parameters
	evtol1_weight 			= 1500.0 # kg
	evtol1_r_rotor_lift		= 1.0 # m
	evtol1_cruise_speed 	= 30.0 # m/s
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
	# Battery parameters
	evtol2_params['battery_rho'] 			= 250.0 # Wh/kg
	evtol2_params['battery_eff'] 			= 0.85
	evtol2_params['battery_max_discharge'] 	= 0.8
	# Design parameters
	evtol2_weight 			= 1500.0 # kg
	evtol2_r_rotor_lift		= 1.0 # m
	evtol2_r_rotor_cruise 	= 1.0 # m
	evtol2_cruise_speed 	= 30.0 # m/s
	evtol2_wing_area 		= 8.0 # m**2
	evtol2_wing_AR 			= 10.0
	evtol2_rotor_J 			= 1.0

	# Constants
	evtol2_params['AoA_cruise']				= 5.0 # deg
	evtol2_params['rho_air'] 				= 1.225 # kg/m**3
	evtol2_params['gravitational_accel'] 	= 9.81 # kg/m**3

	# --- Mission requirements --- #
	n_missions1		= 30
	flight_ranges1 	= np.linspace(50.0, 30000.0, n_missions1) # m
	hover_times1 	= n_missions1 * [240.0] # s
	n_missions2		= 78
	flight_ranges2 	= np.linspace(50.0, 78000.0, n_missions2) # m
	hover_times2 	= n_missions2 * [240.0] # s

	# --- Energy consumption for Wingless Multirotor --- #
	energy_list1 = np.zeros(n_missions1)

	for i in range(n_missions1):
		print(f'Solving Wingless Multirotor; range = {flight_ranges1[i]/1000} km')
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
		indeps.add_output('flight_distance', flight_ranges1[i], units='m')
		indeps.add_output('hover_time', hover_times1[i], units='s')
		indeps.add_output('eVTOL|W_takeoff', evtol1_weight, units='kg')
		indeps.add_output('eVTOL|Cruise_speed', evtol1_cruise_speed, units='m/s')
		indeps.add_output('Rotor|radius_lift', evtol1_r_rotor_lift, units='m')
		indeps.add_output('Rotor|mu', evtol1_rotor_mu)
		
		prob.model.add_subsystem('energy_model',
								  EnergyConsumption(evtol_options=evtol1_params),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		prob.setup(check=False)
		prob.run_model()
		energy_list1[i] = prob.get_val('energy_cnsmp', 'kW*h')[0]

	# --- Energy consumption for Lift+Cruise --- #
	energy_list2 = np.zeros(n_missions2)

	for i in range(n_missions2):
		print(f'Solving Winged Lift+Cruise; range = {flight_ranges2[i]/1000} km')
		prob = om.Problem()
		indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
		indeps.add_output('flight_distance', flight_ranges2[i], units='m')
		indeps.add_output('hover_time', hover_times2[i], units='s')
		indeps.add_output('eVTOL|W_takeoff', evtol2_weight, units='kg')
		indeps.add_output('eVTOL|Cruise_speed', evtol2_cruise_speed, units='m/s')
		indeps.add_output('Rotor|radius_lift', evtol2_r_rotor_lift, units='m')
		indeps.add_output('Rotor|radius_cruise', evtol2_r_rotor_cruise, units='m')
		indeps.add_output('eVTOL|S_wing', evtol2_wing_area, units='m**2')
		indeps.add_output('eVTOL|AR_wing', evtol2_wing_AR)
		indeps.add_output('Rotor|J', evtol2_rotor_J)
		
		prob.model.add_subsystem('energy_model',
								  EnergyConsumption(evtol_options=evtol2_params),
								  promotes_inputs=['*'],
								  promotes_outputs=['*'])

		prob.setup(check=False)
		prob.run_model()
		energy_list2[i] = prob.get_val('energy_cnsmp', 'kW*h')[0]

	plt.plot(flight_ranges1/1000.0, energy_list1, label='Wingless Multirotor')
	plt.plot(flight_ranges2/1000.0, energy_list2, label='Winged Lift+Cruise')
	plt.xlabel('Mission range [km]')
	plt.ylabel('Energy consumption [kWh]')
	plt.title('Energy consumption vs mission range')
	plt.legend(loc='lower right')
	plt.grid(True)
	plt.show()

















		