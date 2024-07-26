from MCEVS.Vehicles.Container import MultirotorEVTOL
from MCEVS.Vehicles.Container import LiftPlusCruiseEVTOL

def StandardMultirotorEVTOL(design_var:dict):

	r_lift_rotor = design_var['r_lift_rotor']

	vehicle = MultirotorEVTOL()
	vehicle.add_component(kind='battery', density=250, efficiency=0.85, max_discharge=0.8)
	vehicle.add_component(kind='fuselage', length=1.8, max_perimeter=6.7)
	vehicle.add_component(kind='landing_gear', strut_length=0.7, ultimate_load_factor=5.7)
	vehicle.add_component(kind='lift_rotor', n_rotor=7, solidity=0.13, radius=r_lift_rotor)
	
	return vehicle

def StandardLiftPlusCruiseEVTOL(design_var:dict):

	wing_area = design_var['wing_area']
	wing_AR = design_var['wing_AR']
	r_lift_rotor = design_var['r_lift_rotor']
	r_propeller = design_var['r_propeller']

	vehicle = LiftPlusCruiseEVTOL()
	vehicle.add_component(kind='battery', density=250, efficiency=0.85, max_discharge=0.8)
	vehicle.add_component(kind='wing', area=wing_area, aspect_ratio=wing_AR, ultimate_load_factor=3.0)
	vehicle.add_component(kind='fuselage', length=1.8, max_perimeter=6.7)
	vehicle.add_component(kind='landing_gear', strut_length=0.7, ultimate_load_factor=5.7)
	vehicle.add_component(kind='lift_rotor', n_rotor=6, solidity=0.13, radius=r_lift_rotor)
	vehicle.add_component(kind='propeller', n_propeller=1, solidity=0.13, radius=r_propeller)
	
	return vehicle