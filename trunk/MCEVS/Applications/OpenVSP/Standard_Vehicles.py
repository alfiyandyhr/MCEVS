import openvsp as vsp
from MCEVS.Applications.OpenVSP.Components.Fuselage import NASA_LPC_Fuselage
from MCEVS.Applications.OpenVSP.Components.Wing import NASA_LPC_Wing
from MCEVS.Applications.OpenVSP.Components.Tail import NASA_LPC_Horizontal_Tail, NASA_LPC_Vertical_Tail
from MCEVS.Applications.OpenVSP.Components.Landing_Gear import NASA_LPC_Landing_Gear
from MCEVS.Applications.OpenVSP.Components.Rotor import NASA_LPC_Lift_Rotor, NASA_LPC_Propeller
from MCEVS.Applications.OpenVSP.Components.Payload import Human
from MCEVS.Applications.OpenVSP.Components.Boom import NASA_LPC_Boom

def create_NASA_LiftPlusCruise_vsp3(vehicle:object):

	# Unpacking parameters
	n_pax 			= vehicle.fuselage.number_of_passenger
	l_fuse  		= vehicle.fuselage.length
	d_fuse_max 		= vehicle.fuselage.max_diameter
	wing_S 			= vehicle.wing.area
	wing_AR 		= vehicle.wing.aspect_ratio
	l_strut 		= vehicle.landing_gear.strut_length
	n_lift_rotor 	= vehicle.lift_rotor.n_rotor
	n_blade_rotor 	= vehicle.lift_rotor.n_blade
	n_propeller 	= vehicle.propeller.n_propeller
	n_blade_prop 	= vehicle.propeller.n_blade
	r_lift_rotor 	= vehicle.lift_rotor.radius
	r_propeller 	= vehicle.propeller.radius

	# print(n_pax, l_fuse, d_fuse_max, wing_S, wing_AR, l_strut)
	# print(n_lift_rotor, n_blade_rotor, n_propeller, n_blade_prop)
	# print(r_lift_rotor, r_propeller)

	Human(N_PAX=n_pax)
	NASA_LPC_Fuselage(l_fuse, d_fuse_max)
	NASA_LPC_Wing(area=wing_S, aspect_ratio=wing_AR, l_fuse=l_fuse)
	NASA_LPC_Horizontal_Tail(area=2.0, aspect_ratio=2.0, l_fuse=l_fuse)
	NASA_LPC_Vertical_Tail(area=2.5, aspect_ratio=1.2, l_fuse=l_fuse)
	NASA_LPC_Landing_Gear(l_strut=l_strut, l_fuse=l_fuse, d_fuse_max=d_fuse_max)
	NASA_LPC_Lift_Rotor(n_lift_rotor=n_lift_rotor, n_blade=n_blade_rotor, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR)
	NASA_LPC_Propeller(n_propeller=n_propeller, n_blade=n_blade_prop, r_propeller=r_propeller, l_fuse=l_fuse)
	NASA_LPC_Boom(n_lift_rotor=n_lift_rotor, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR)
	vsp.WriteVSPFile('vehicle.vsp3')