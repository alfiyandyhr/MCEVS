import openvsp as vsp
from MCEVS.Applications.OpenVSP.Components.Fuselage import NASA_QR_Fuselage, NASA_LPC_Fuselage
from MCEVS.Applications.OpenVSP.Components.Wing import NASA_LPC_Wing
from MCEVS.Applications.OpenVSP.Components.Tail import NASA_LPC_Horizontal_Tail, NASA_LPC_Vertical_Tail
from MCEVS.Applications.OpenVSP.Components.Landing_Gear import NASA_QR_Landing_Gear, NASA_LPC_Landing_Gear
from MCEVS.Applications.OpenVSP.Components.Rotor import NASA_QR_Lift_Rotor, NASA_LPC_Lift_Rotor, NASA_LPC_Propeller
from MCEVS.Applications.OpenVSP.Components.Payload import Human
from MCEVS.Applications.OpenVSP.Components.Boom import NASA_QR_Boom, NASA_LPC_Boom

def create_NASA_QuadRotor_vsp3(fname:str, vehicle:object):

	# Unpacking parameters
	config  		= vehicle.configuration
	n_pax 			= vehicle.fuselage.number_of_passenger
	l_fuse  		= vehicle.fuselage.length
	d_fuse_max 		= vehicle.fuselage.max_diameter
	l_strut 		= vehicle.landing_gear.strut_length
	n_lift_rotor 	= vehicle.lift_rotor.n_rotor
	n_blade_rotor 	= vehicle.lift_rotor.n_blade
	r_lift_rotor 	= vehicle.lift_rotor.radius

	human_ids = Human(N_PAX=n_pax, config=config)
	fuse_id   = NASA_QR_Fuselage(l_fuse=l_fuse, d_fuse_max=d_fuse_max)
	boom_ids  = NASA_QR_Boom(n_lift_rotor=4, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, d_fuse_max=d_fuse_max, fuse_id=fuse_id)
	rotor_ids = NASA_QR_Lift_Rotor(n_lift_rotor=n_lift_rotor, r_lift_rotor=r_lift_rotor, n_blade=n_blade_rotor, l_fuse=l_fuse, d_fuse_max=d_fuse_max, boom_ids=boom_ids)
	lg_ids    = NASA_QR_Landing_Gear(l_strut=l_strut, l_fuse=l_fuse, d_fuse_max=d_fuse_max)
	vsp.WriteVSPFile(fname)
	vsp.ClearVSPModel()

def create_NASA_LiftPlusCruise_vsp3(fname:str, vehicle:object):

	# Unpacking parameters
	config  		= vehicle.configuration
	n_pax 			= vehicle.fuselage.number_of_passenger
	l_fuse  		= vehicle.fuselage.length
	d_fuse_max 		= vehicle.fuselage.max_diameter
	wing_S 			= vehicle.wing.area
	wing_AR 		= vehicle.wing.aspect_ratio
	htail_S 		= vehicle.horizontal_tail.area
	htail_AR  		= vehicle.horizontal_tail.aspect_ratio
	vtail_S  		= vehicle.vertical_tail.area
	vtail_AR		= vehicle.vertical_tail.aspect_ratio
	l_strut 		= vehicle.landing_gear.strut_length
	n_lift_rotor 	= vehicle.lift_rotor.n_rotor
	n_blade_rotor 	= vehicle.lift_rotor.n_blade
	n_propeller 	= vehicle.propeller.n_propeller
	n_blade_prop 	= vehicle.propeller.n_blade
	r_lift_rotor 	= vehicle.lift_rotor.radius
	r_propeller 	= vehicle.propeller.radius

	Human(N_PAX=n_pax, config=config)
	NASA_LPC_Fuselage(l_fuse, d_fuse_max)
	NASA_LPC_Wing(area=wing_S, aspect_ratio=wing_AR, l_fuse=l_fuse)
	NASA_LPC_Horizontal_Tail(area=htail_S, aspect_ratio=htail_AR, l_fuse=l_fuse)
	NASA_LPC_Vertical_Tail(area=vtail_S, aspect_ratio=vtail_AR, l_fuse=l_fuse)
	NASA_LPC_Landing_Gear(l_strut=l_strut, l_fuse=l_fuse, d_fuse_max=d_fuse_max)
	NASA_LPC_Lift_Rotor(n_lift_rotor=n_lift_rotor, n_blade=n_blade_rotor, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR)
	NASA_LPC_Propeller(n_propeller=n_propeller, n_blade=n_blade_prop, r_propeller=r_propeller, l_fuse=l_fuse)
	NASA_LPC_Boom(n_lift_rotor=n_lift_rotor, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR)
	vsp.WriteVSPFile(fname)
	vsp.ClearVSPModel()