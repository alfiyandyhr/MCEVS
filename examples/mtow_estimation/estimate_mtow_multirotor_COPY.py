import numpy as np
import openmdao.api as om

from MCEVS.Weights.Groups import MTOWEstimation

if __name__ == '__main__':

	# ============================================ #
	# ------------- eVTOL: Multirotor ------------ #
	# ============================================ #
	# General parameters
	evtol_params = {}
	evtol_params['evtol_config'] 			= 'multirotor'
	evtol_params['N_rotors_lift'] 			= 7
	evtol_params['rotor_lift_solidity'] 	= 0.13
	evtol_params['hover_FM'] 				= 0.75
	# Fuselage parameters
	evtol_params['n_pax']					= 4 # number of passengers (including pilot)
	evtol_params['l_fuse']					= 6.7 # m
	evtol_params['d_fuse_max']				= 1.8 # m
	# Landing gear parameters
	evtol_params['l_sm']					= 0.7 # m
	evtol_params['n_ult_lg']				= 5.7
	# Battery parameters
	evtol_params['battery_rho'] 			= 250.0 # Wh/kg
	evtol_params['battery_eff'] 			= 0.85
	evtol_params['battery_max_discharge'] 	= 0.8

	# Constants
	evtol_params['rho_air'] 				= 1.225 # kg/m**3
	evtol_params['gravitational_accel'] 	= 9.81 # kg/m**3
	evtol_params['technology_factor']		= 0.8 # composite reduces 20% of weight

	# --- Mission requirements --- #
	payload_weight	= 400.0 # kg
	flight_range	= 30000.0 # m
	hover_time	 	= 240.0 # s

	# --- Design variable values --- #
	cruise_speed	= 30.0 # m/s
	r_rotor_lift	= 1.0 # m
	rotor_mu	 	= 0.3
	# ============================================ #

	# --- Setting up as an OpenMDAO problem --- #
	prob = om.Problem()
	indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
	indeps.add_output('payload_weight', payload_weight, units='kg')
	indeps.add_output('flight_distance', flight_range, units='m')
	indeps.add_output('hover_time', hover_time, units='s')
	indeps.add_output('eVTOL|Cruise_speed', cruise_speed, units='m/s')
	indeps.add_output('Rotor|radius_lift', r_rotor_lift, units='m')
	indeps.add_output('Rotor|mu', rotor_mu, units=None)
	
	prob.model.add_subsystem('mtow_estimation',
							  MTOWEstimation(evtol_options=evtol_params, use_solver=True),
							  promotes_inputs=['*'],
							  promotes_outputs=['*'])

	prob.setup(check=False)
	prob.run_model()
	# prob.check_partials(compact_print=True, show_only_incorrect=True)
	# mtow_list1[i = prob.get_val('eVTOL|W_takeoff')[0]

	# get weights
	W_takeoff = prob.get_val('eVTOL|W_takeoff', 'kg')
	W_payload = prob.get_val('payload_weight', 'kg')
	W_battery = prob.get_val('Weights|Battery', 'kg')
	W_rotors = prob.get_val('Weights|Rotors', 'kg')
	W_motors = prob.get_val('Weights|Motors', 'kg')
	W_controllers = prob.get_val('Weights|MotorControllers', 'kg')
	W_fuselage = prob.get_val('Weights|Fuselage', 'kg')
	W_landing_gear = prob.get_val('Weights|Landing_gear', 'kg')
	W_avionics = prob.get_val('Weights|Avionics', 'kg')
	W_flight_control = prob.get_val('Weights|Flight_control', 'kg')
	W_anti_icing = prob.get_val('Weights|Anti_icing', 'kg')
	W_furnishings = prob.get_val('Weights|Furnishings', 'kg')
	W_residual = prob.get_val('W_residual', 'kg')


	# --- print results ---
	print('--------------------------------------------')
	print('--- problem settings ---')
	print('  eVTOL parameters echo:', evtol_params)
	print('  payload weight [kg]:', list(W_payload))
	print('  flight range [m]   :', list(prob.get_val('flight_distance', 'm')))
	print('  hovering time [s]  :', list(prob.get_val('hover_time', 's')))
	print('\n--- design optimization results ---')
	print('Design variables')
	print('  lifting rotor radius [m] :', list(prob.get_val('Rotor|radius_lift', 'm')))
	print('  cruise speed [m/s]       :', list(prob.get_val('eVTOL|Cruise_speed', 'm/s')))
	print('  edgewise advance ratio mu:', list(prob.get_val('Rotor|mu')))
	print('Component weights [kg]')
	print('  total weight :', list(W_takeoff))
	print('  payload      :', list(W_payload))
	print('  battery      :', list(W_battery))
	print('  rotors       :', list(W_rotors))
	print('  motors 	  :', list(W_motors))
	print('  controllers  :', list(W_controllers))
	print('  fuselage 	  :', list(W_fuselage))
	print('  landing_gear :', list(W_landing_gear))
	print('  avionics     :', list(W_avionics))
	print('  flight_ctrl  :', list(W_flight_control))
	print('  anti_icing   :', list(W_anti_icing))
	print('  furnishings  :', list(W_furnishings))
	print('Performances')
	print('  power in hover: [W] :', list(prob.get_val('power_hover', 'W')))
	print('  power in cruise: [W]:', list(prob.get_val('power_forward', 'W')))
	print('Sanity check: W_residual [kg]:', list(prob.get_val('W_residual', 'kg')), ' = 0?')
	print('--------------------------------------------')