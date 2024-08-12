from MCEVS.Vehicles.Container import MultirotorEVTOL
from MCEVS.Vehicles.Container import LiftPlusCruiseEVTOL

def StandardMultirotorEVTOL(design_var:dict, mtow=None):

	r_lift_rotor 	= design_var['r_lift_rotor']
	rotor_mu		= design_var['rotor_advance_ratio']

	vehicle = MultirotorEVTOL(mtow=mtow)
	vehicle.add_component(kind='battery', density=250, efficiency=0.85, max_discharge=0.8)
	vehicle.add_component(kind='fuselage', length=9.0, max_diameter=1.8, number_of_passenger=4, technology_factor=0.8)
	vehicle.add_component(kind='landing_gear', strut_length=0.7, ultimate_load_factor=5.7, technology_factor=0.8)
	vehicle.add_component(kind='lift_rotor', n_rotor=7, n_blade=4, solidity=0.13, radius=r_lift_rotor, figure_of_merit=0.75, advance_ratio=rotor_mu, technology_factor=0.8)
	
	return vehicle

def StandardLiftPlusCruiseEVTOL(design_var:dict, mtow=None):

	wing_area 		= design_var['wing_area']
	wing_AR 		= design_var['wing_aspect_ratio']
	r_lift_rotor 	= design_var['r_lift_rotor']
	r_propeller 	= design_var['r_propeller']
	propeller_J 	= design_var['propeller_advance_ratio']

	vehicle = LiftPlusCruiseEVTOL(mtow=mtow)
	vehicle.add_component(kind='battery', density=250, efficiency=0.85, max_discharge=0.8)
	vehicle.add_component(kind='wing', area=wing_area, aspect_ratio=wing_AR, ultimate_load_factor=3.0, technology_factor=0.8)
	vehicle.add_component(kind='fuselage', length=9.0, max_diameter=1.8, number_of_passenger=4, technology_factor=0.8)
	vehicle.add_component(kind='landing_gear', strut_length=0.3, ultimate_load_factor=5.7, technology_factor=0.8)
	vehicle.add_component(kind='lift_rotor', n_rotor=8, n_blade=2, solidity=0.13, radius=r_lift_rotor, figure_of_merit=0.75, technology_factor=0.8)
	vehicle.add_component(kind='propeller', n_propeller=1, n_blade=4, solidity=0.13, radius=r_propeller, advance_ratio=propeller_J, technology_factor=0.8)
	# vehicle.add_component(kind='')

	return vehicle