# import numpy a30np
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

	# --- Mission requirements --- #
	payload_weight	= 400.0 # kg
	flight_range	= 30000.0 # m
	hover_time	 	= 240.0 # s
	# ============================================ #

	# --- Setting up as an OpenMDAO problem --- #

	prob = om.Problem()

	# Define model inputs
	indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])

	# Mission requirements
	indeps.add_output('payload_weight', payload_weight, units='kg')
	indeps.add_output('flight_distance', flight_range, units='m')
	indeps.add_output('hover_time', hover_time, units='s')

	# Design variables (and their initial gueses)
	# indeps.add_output('eVTOL|W_takeoff', 1058.6687, units='kg')
	indeps.add_output('eVTOL|Cruise_speed', 30.0, units='m/s')
	indeps.add_output('Rotor|radius_lift', 1.0, units='m')
	indeps.add_output('Rotor|mu', 0.3, units=None)

	# eVTOL MTOW Estimation model
	prob.model.add_subsystem('evtol_weight_model',
							  MTOWEstimation(evtol_options=evtol_params, use_solver=True),
							  promotes_inputs=['*'],
							  promotes_outputs=['*'])

	# --- Optimization problem --- #
	# Design variables and their lower/upper bounds
	# prob.model.add_design_var('eVTOL|W_takeoff', lower=500.0, upper=2500.0, units='kg')
	prob.model.add_design_var('eVTOL|Cruise_speed', lower=20.0, upper=60.0, units='m/s')
	prob.model.add_design_var('Rotor|radius_lift', lower=0.5, upper=1.5, units='m')
	prob.model.add_design_var('Rotor|mu', lower=0.01, upper=0.5)

	# Constraints
	# prob.model.add_constraint('W_residual', lower=0.0, upper=0.0)
	prob.model.add_constraint('disk_loading_hover', upper=600.0, units='N/m**2')
	prob.model.add_constraint('disk_loading_cruise', upper=600.0, units='N/m**2')
	# in cruise. CT / solidity <= 0.14 to avoid too high blade loading
	prob.model.add_constraint('Rotor|Ct', lower=0.0, upper=0.30 * evtol_params['rotor_lift_solidity'])

	# Objective
	prob.model.add_objective('eVTOL|W_takeoff')

	# Optimizer settings
	prob.driver = om.ScipyOptimizeDriver(optimizer='SLSQP', tol=1e-8, disp=True)
	prob.driver.recording_options['includes'] = ['*']
	prob.driver.recording_options['record_objectives'] = True
	prob.driver.recording_options['record_constraints'] = True
	prob.driver.recording_options['record_desvars'] = True
	prob.driver.recording_options['record_inputs'] = True
	prob.driver.recording_options['record_outputs'] = True
	prob.driver.recording_options['record_residuals'] = True
	prob.driver.add_recorder(om.SqliteRecorder("multirotor_opt_cases.sql"))
	
	# Run optimization
	prob.setup(check=False)
	prob.run_driver()
	# prob.run_model()

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
	print('  power in hover [W] :', list(prob.get_val('power_hover', 'W')))
	print('  power in cruise [W]:', list(prob.get_val('power_forward', 'W')))
	print('  disk loading rotor_lift [N/m**2]:', list(prob.get_val('disk_loading_hover', 'N/m**2')))
	print('  disk loading rotor_cruise [N/m**2]:', list(prob.get_val('disk_loading_cruise', 'N/m**2')))
	print('  thrust coefficient/solidity:', list(prob.get_val('Rotor|Ct')/evtol_params['rotor_lift_solidity']))
	print('Sanity check: W_residual [kg]:', list(prob.get_val('W_residual', 'kg')), ' = 0?')
	print('--------------------------------------------')
		

