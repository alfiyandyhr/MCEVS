import numpy as np
import openvsp as vsp
from MCEVS.Applications.OpenVSP.Components.Fuselage import NASA_QR_Fuselage, NASA_LPC_Fuselage
from MCEVS.Applications.OpenVSP.Components.Wing import NASA_LPC_Wing
from MCEVS.Applications.OpenVSP.Components.Tail import NASA_LPC_Horizontal_Tail, NASA_LPC_Vertical_Tail
from MCEVS.Applications.OpenVSP.Components.Landing_Gear import NASA_QR_Landing_Gear, NASA_LPC_Landing_Gear
from MCEVS.Applications.OpenVSP.Components.Rotor import NASA_QR_Lift_Rotor, NASA_LPC_Lift_Rotor, NASA_LPC_Propeller
from MCEVS.Applications.OpenVSP.Components.Boom import NASA_QR_Boom, NASA_LPC_Boom

def calc_wetted_area(vehicle:object):

	# Unpacking parameters
	config  		= vehicle.configuration
	n_pax 			= vehicle.fuselage.number_of_passenger
	l_fuse  		= vehicle.fuselage.length
	d_fuse_max 		= vehicle.fuselage.max_diameter
	l_strut 		= vehicle.landing_gear.strut_length
	n_lift_rotor 	= vehicle.lift_rotor.n_rotor
	n_blade_rotor 	= vehicle.lift_rotor.n_blade
	r_lift_rotor 	= vehicle.lift_rotor.radius

	if config == 'LiftPlusCruise':
		wing_S 			= vehicle.wing.area
		wing_AR 		= vehicle.wing.aspect_ratio
		htail_S 		= vehicle.horizontal_tail.area
		htail_AR  		= vehicle.horizontal_tail.aspect_ratio
		vtail_S  		= vehicle.vertical_tail.area
		vtail_AR		= vehicle.vertical_tail.aspect_ratio
		n_propeller 	= vehicle.propeller.n_propeller
		n_blade_prop 	= vehicle.propeller.n_blade
		r_propeller 	= vehicle.propeller.radius

	if config == 'LiftPlusCruise':
		NASA_LPC_Fuselage(l_fuse, d_fuse_max)
		NASA_LPC_Wing(area=wing_S, aspect_ratio=wing_AR, l_fuse=l_fuse)
		NASA_LPC_Horizontal_Tail(area=htail_S, aspect_ratio=htail_AR, l_fuse=l_fuse)
		NASA_LPC_Vertical_Tail(area=vtail_S, aspect_ratio=vtail_AR, l_fuse=l_fuse)
		NASA_LPC_Boom(n_lift_rotor=n_lift_rotor, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR)
		NASA_LPC_Lift_Rotor(n_lift_rotor=n_lift_rotor, n_blade=n_blade_rotor, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR)
		NASA_LPC_Propeller(n_propeller=n_propeller, n_blade=n_blade_prop, r_propeller=r_propeller, l_fuse=l_fuse)
		NASA_LPC_Landing_Gear(l_strut=l_strut, l_fuse=l_fuse, d_fuse_max=d_fuse_max)
		mesh_id	= vsp.ComputeCompGeom(vsp.SET_ALL, False, 0)
		comp_res_id = vsp.FindLatestResultsID('Comp_Geom')
		double_arr = vsp.GetDoubleResults(comp_res_id, 'Wet_Area')

		res = {}
		res['Fuselage'] 			= double_arr[0]
		res['Wing']					= double_arr[1]
		res['HTail']				= double_arr[2]
		res['VTail']				= double_arr[3]
		res['Boom_1']				= double_arr[4]
		res['Boom_2']				= double_arr[5]
		res['Boom_3']				= double_arr[6]
		res['Boom_4']				= double_arr[7]
		res['RotorHub_1'] 			= double_arr[8]
		res['RotorHub_2'] 			= double_arr[9]
		res['RotorHub_3'] 			= double_arr[10]
		res['RotorHub_4'] 			= double_arr[11]
		res['RotorBlade_1_1']		= double_arr[12]
		res['RotorBlade_1_2']		= double_arr[13]
		res['RotorBlade_2_1']		= double_arr[15]
		res['RotorBlade_2_2']		= double_arr[16]
		res['RotorBlade_3_1']		= double_arr[18]
		res['RotorBlade_3_2']		= double_arr[19]
		res['RotorBlade_4_1']		= double_arr[21]
		res['RotorBlade_4_2']		= double_arr[22]
		res['PropellerHub_1'] 		= double_arr[24]
		res['PropellerBlade_1_1']	= double_arr[25]
		res['PropellerBlade_1_2']	= double_arr[26]
		res['PropellerBlade_1_3']	= double_arr[27]
		res['PropellerBlade_1_4']	= double_arr[28]
		res['NoseStrut_LG']			= double_arr[34]
		res['MainStrut_LG_1']		= double_arr[35]
		res['MainStrut_LG_2']		= double_arr[36]
		res['NoseWheel_LG']			= double_arr[37]
		res['MainWheel_LG_1']		= double_arr[38]
		res['MainWheel_LG_2']		= double_arr[39]		

		return res
	