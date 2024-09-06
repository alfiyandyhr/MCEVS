from MCEVS.Vehicles.Container import MultirotorEVTOL
from MCEVS.Vehicles.Container import LiftPlusCruiseEVTOL

def StandardMultirotorEVTOL(design_var:dict, n_pax=4, mtow=None):

	r_lift_rotor 	= design_var['r_lift_rotor']
	rotor_mu		= design_var['rotor_advance_ratio']

	if n_pax == 4:
		vehicle = MultirotorEVTOL(mtow=mtow)
		vehicle.add_component(kind='battery', density=250, efficiency=0.85, max_discharge=0.8)
		vehicle.add_component(kind='fuselage', length=5.2, max_diameter=1.8, number_of_passenger=n_pax, technology_factor=0.8)
		vehicle.add_component(kind='landing_gear', strut_length=0.3, ultimate_load_factor=5.7, technology_factor=0.8)
		vehicle.add_component(kind='lift_rotor', n_rotor=4, n_blade=5, solidity=0.13, radius=r_lift_rotor, figure_of_merit=0.75, advance_ratio=rotor_mu, technology_factor=0.8)
	
	return vehicle

def StandardLiftPlusCruiseEVTOL(design_var:dict, n_pax=4, mtow=None):

	wing_area 		= design_var['wing_area']
	wing_AR 		= design_var['wing_aspect_ratio']
	r_lift_rotor 	= design_var['r_lift_rotor']
	r_propeller 	= design_var['r_propeller']
	propeller_J 	= design_var['propeller_advance_ratio']

	if n_pax == 4:
		l_fuse 		= 9.0
		l_boom 		= 0.8*8.0/30.0*l_fuse/1.6*r_lift_rotor if r_lift_rotor>=1.6 else 0.8*8.0/30.0*l_fuse
		d_boom 		= 1.0/30.0*l_fuse
		l_rotor_hub = 1.0/30.0*l_fuse
		d_rotor_hub = 2.25/10.0*r_lift_rotor
		l_prop_hub 	= 1.0/30.0*l_fuse
		d_prop_hub 	= 1.5/9.0*r_propeller

		vehicle = LiftPlusCruiseEVTOL(mtow=mtow)
		vehicle.add_component(kind='battery', density=250, efficiency=0.85, max_discharge=0.8)
		vehicle.add_component(kind='wing', area=wing_area, aspect_ratio=wing_AR, thickness_to_chord_ratio=0.16998, ultimate_load_factor=3.0, technology_factor=0.8)
		vehicle.add_component(kind='fuselage', length=l_fuse, max_diameter=1.8, number_of_passenger=n_pax, technology_factor=0.8)
		vehicle.add_component(kind='landing_gear', strut_length=0.3, ultimate_load_factor=5.7, technology_factor=0.8)
		vehicle.add_component(kind='boom', length=l_boom, max_diameter=d_boom, number_of_booms=4)
		vehicle.add_component(kind='lift_rotor', n_rotor=4, n_blade=2, solidity=0.13, radius=r_lift_rotor, hub_length=l_rotor_hub, hub_max_diameter=d_rotor_hub, figure_of_merit=0.75, technology_factor=0.8)
		vehicle.add_component(kind='propeller', n_propeller=1, n_blade=4, solidity=0.13, radius=r_propeller, advance_ratio=propeller_J, hub_length=l_prop_hub, hub_max_diameter=d_prop_hub, technology_factor=0.8)
		vehicle.add_component(kind='horizontal_tail', area=2.0, aspect_ratio=2.0, taper_ratio=0.6, max_root_thickness=0.15*1.25, thickness_to_chord_ratio=0.15, technology_factor=0.8)
		vehicle.add_component(kind='vertical_tail', area=2.5, aspect_ratio=1.2, max_root_thickness=0.12*2.92, sweep_angle=35, thickness_to_chord_ratio=0.136, technology_factor=0.8)

	return vehicle