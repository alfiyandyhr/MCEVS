import openmdao.api as om
from MCEVS.Analyses.Weights.Groups import MTOWEstimation

def size_vehicle(vehicle:object, mission:object):
	"""
	Size a vehicle, in other words, converge the vehicle parameters so that the physics works.
	"""

	# --- Setting up as an OpenMDAO problem --- #
	prob = om.Problem()
	indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
	indeps.add_output('payload_weight', payload_weight, units='kg')
	indeps.add_output('flight_distance', flight_range, units='m')
	indeps.add_output('hover_time', hover_time, units='s')
	indeps.add_output('eVTOL|Cruise_speed', cruise_speed, units='m/s')
	indeps.add_output('eVTOL|S_wing', wing_area, units='m**2')
	indeps.add_output('eVTOL|AR_wing', wing_AR, units=None)
	indeps.add_output('Rotor|radius_lift', r_rotor_lift, units='m')
	indeps.add_output('Rotor|radius_cruise', r_rotor_cruise, units='m')
	indeps.add_output('Rotor|J', rotor_J, units=None)
	
	prob.model.add_subsystem('mtow_estimation',
							  MTOWEstimation(evtol_options=evtol_params, use_solver=True),
							  promotes_inputs=['*'],
							  promotes_outputs=['*'])

	prob.setup(check=False)
	prob.run_model()
