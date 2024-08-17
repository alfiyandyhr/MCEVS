import openvsp as vsp
import numpy as np


def NASA_QR_Lift_Rotor(n_lift_rotor=4, r_lift_rotor=9.159, n_blade=5, l_fuse=21.0, d_fuse_max=6.745500, boom_ids=[None]):

	# --- Creating rotor hubs --- #

	# Baseline params
	l_hub  			= 1.7/21.0*l_fuse
	X  				= [  0.000000,  0.343827,  0.500000,  0.759227,  1.000000 ]
	Y  				= [  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]
	Z  				= [  0.000000,  0.000000,  0.000000,  0.010870,  0.000000 ]
	TopLAngle 		= [  90.00000,  28.64348,  0.000000, -34.07000, -90.00000 ]
	TopLSlew		= [  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]
	TopLStrength 	= [  1.549250,  1.000000,  1.000000,  1.277883,  1.122283 ]
	TopLCurve 		= [  2.802916, -11.44160, -0.418898, -1.003574,  1.247315 ]
	RightLAngle 	= [  90.00000,  34.20000,  0.000000, -25.20000, -90.00000 ]
	RightLSlew 		= [  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]
	RightLStrength 	= [  1.440000,  1.000000,  1.000000,  1.000000,  1.440000 ]
	RightLCurve 	= [  3.373307, -9.347744, -0.830599, -1.271703,  2.172973 ]
	BottomLAngle 	= [  90.00000,  28.64348,  0.000000, -34.07000, -90.00000 ]
	BottomLStrength = [  1.549250,  1.000000,  1.000000,  1.277883,  1.122283 ]
	BottomLSlew		= [  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]
	BottomLCurve 	= [  2.802916, -11.44160, -0.418898, -0.548002,  1.212800 ]

	if n_lift_rotor == 4:
		Ellipse_Height	= [ 0.000000, 2.071818, 2.200000, 1.868182, 0.000000 ]
		Ellipse_Width	= [ 0.000000, 1.517045, 1.700000, 1.583316, 0.000000 ]
		Y_Rel_Rotations = [ 0.0, 0.0, 0.0, 0.0 ]

	hub_ids = []
	for i in range(int(n_lift_rotor)):
		hub_id 		= vsp.AddGeom('FUSELAGE', boom_ids[i])
		hub_ids.append(hub_id)
		hub_surf 	= vsp.GetXSecSurf(hub_id, 0)
		xsec_num	= vsp.GetNumXSec(hub_surf)
		vsp.SetParmVal( hub_id, 'Length', 'Design', l_hub)
		vsp.SetParmVal( hub_id, 'X_Rel_Location', 		'XForm', 	0.0    			   )
		vsp.SetParmVal( hub_id, 'Y_Rel_Location', 		'XForm', 	0.0    			   )
		vsp.SetParmVal( hub_id, 'Z_Rel_Location', 		'XForm', 	0.0    			   )
		vsp.SetParmVal( hub_id, 'Y_Rel_Rotation', 		'XForm',    Y_Rel_Rotations[i] )
		vsp.SetParmVal( hub_id, 'Sym_Planar_Flag', 		'Sym', 		0 	   			   )
		vsp.SetParmVal( hub_id, 'Trans_Attach_Flag', 	'Attach',  	2.0 			   )
		vsp.SetParmVal( hub_id, 'Rots_Attach_Flag',  	'Attach',  	1.0 			   )
		vsp.SetParmVal( hub_id, 'U_Attach_Location', 	'Attach',  	1.0 			   )
		vsp.SetParmVal( hub_id, 'V_Attach_Location', 	'Attach',  	0.5 			   )

		# Change shape of XSecs, and set height + width
		for j in range(1, xsec_num-1):
			xsec = vsp.GetXSec(hub_surf, j)
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Ellipse_Height'), Ellipse_Height[j]/21.0*l_fuse )
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Ellipse_Width'),  Ellipse_Width[j]/21.0*l_fuse  )

		# Set locations and skinning params
		for j in range(xsec_num):
			xsec = vsp.GetXSec(hub_surf, j)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'XLocPercent'),		X[j]				)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'YLocPercent'), 		Y[j]				)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'ZLocPercent'), 		Z[j]				)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TBSym'), 			0.0					)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLAngle'), 		TopLAngle[j]		)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLSlew'), 			TopLSlew[j]			)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLStrength'), 		TopLStrength[j]		)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'TopLCurve'), 		TopLCurve[j]		)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLAngle'), 		RightLAngle[j]		)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLSlew'), 		RightLSlew[j]		)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLStrength'), 	RightLStrength[j]	)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'RightLCurve'), 		RightLCurve[j]		)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLAngle'), 		BottomLAngle[j]		)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLStrength'), 	BottomLStrength[j]	)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLSlew'), 		BottomLSlew[j]		)
			vsp.SetParmVal(		vsp.GetXSecParm(xsec, 'BottomLCurve'), 		BottomLCurve[j]		)


	# --- Creating rotor blades --- #
	Y_Rel_Rotations = [90.0, -90.0, 90.0, -90.0]
	for i in range(int(n_lift_rotor)):
		rotor_id = vsp.AddGeom('PROP', hub_ids[i])
		vsp.SetParmVal(rotor_id, 'PropMode', 'Design', 1)
		vsp.SetParmVal(rotor_id, 'Diameter', 'Design', 2*r_lift_rotor)
		vsp.SetParmVal(rotor_id, 'NumBlade', 'Design', n_blade)

		vsp.SetParmVal( rotor_id, 	'X_Rel_Location', 	'XForm', 	l_hub/2 		   )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Location', 	'XForm', 	0.00 			   )
		vsp.SetParmVal( rotor_id, 	'Z_Rel_Location', 	'XForm', 	0.00 			   )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Rotation', 	'XForm', 	Y_Rel_Rotations[i] )
		vsp.SetParmVal( rotor_id, 	'Sym_Planar_Flag', 	'Sym', 		0    			   )

		vsp.SetParmVal( rotor_id, 	'Trans_Attach_Flag', 'Attach', 1.0)
		vsp.SetParmVal( rotor_id, 	'Rots_Attach_Flag',  'Attach', 1.0)
		

def NASA_LPC_Lift_Rotor(n_lift_rotor=8, n_blade=2, r_lift_rotor=5.0, l_fuse=30.0, wing_AR=12.12761, wing_S=210.27814):

	# --- Creating rotor hubs --- #

	# Parameters
	l_hub  = 1.0/30.0*l_fuse
	# d_hub  = 0.8*2.25/30.0*l_fuse
	d_hub  = 2.25/10.0*r_lift_rotor
	l_boom  = 0.8*8.0/30.0*l_fuse/1.6*r_lift_rotor if r_lift_rotor>=1.6 else 0.8*8.0/30.0*l_fuse
	d_boom = 1.0/30.0*l_fuse
	b = np.sqrt(wing_S * wing_AR)	# wing span
	wing_rc = 2.0 * ((5.4432819 + 4.1308305)/2 * 14.745022 / 210.27814 * wing_S) / (14.745022/25.26*b/2) / (1 + 4.1308305/5.4432819)
	wing_c_loc = 2.0 * ((4.130830 + 3.304664)/2 * 6.626930 / 210.27814 * wing_S) / (6.6269300/25.26*b/2) / (1 + 3.304664/4.130830)
	t_per_c = 0.17 # LS-417 airfoil

	wing_X0 = (8.889-1.889)/30*l_fuse
	wing_Z0 = (8.499-4.249)/30*l_fuse

	if n_lift_rotor == 4:
		X_Rel_Locations = [ wing_X0 - (l_boom - 0.5*wing_rc) + 0.5*d_hub,
							wing_X0 + 0.5*wing_rc + l_boom - 0.5*d_hub ]
		Y_Rel_Locations = [ 0.25*b, 0.25*b ]
		Z_Rel_Locations = (wing_Z0 - (t_per_c * wing_c_loc) + l_hub + 0.5*d_boom) * np.ones(2)

	if n_lift_rotor == 8:
		X_Rel_Locations = [ wing_X0 - (l_boom - 0.5*wing_rc) + 0.5*d_hub,
							wing_X0 + 0.5*wing_rc + l_boom - 0.5*d_hub,
							wing_X0 - (l_boom - 0.5*wing_rc) + 0.5*d_hub,
							wing_X0 + 0.5*wing_rc + l_boom - 0.5*d_hub ]
		Y_Rel_Locations = [ 14.745022/2/50.52*b,
							14.745022/2/50.52*b,
							(14.745022+6.626930/2)/50.52*b,
							(14.745022+6.626930/2)/50.52*b ]
		Z_Rel_Locations = (wing_Z0 - (t_per_c * wing_c_loc) + l_hub + 0.5*d_boom) * np.ones(4)

	for i in range(int(n_lift_rotor/2)):
		hub_id 	= vsp.AddGeom('FUSELAGE')
		hub_surf 	= vsp.GetXSecSurf(hub_id, 0)
		xsec_num	= vsp.GetNumXSec(hub_surf)
		vsp.SetParmVal( hub_id, 'Length', 'Design', l_hub)
		vsp.SetParmVal( hub_id, 'X_Rel_Location', 	'XForm', 	X_Rel_Locations[i] )
		vsp.SetParmVal( hub_id, 'Y_Rel_Location', 	'XForm', 	Y_Rel_Locations[i] )
		vsp.SetParmVal( hub_id, 'Z_Rel_Location', 	'XForm', 	Z_Rel_Locations[i] )
		vsp.SetParmVal( hub_id, 'Y_Rel_Rotation', 	'XForm', 	90.0 			   )
		vsp.SetParmVal( hub_id, 'Sym_Planar_Flag', 	'Sym', 		2 				   )

		# Change shape of XSecs, and set height + width
		for i in range(1, xsec_num-1):
			vsp.ChangeXSecShape(hub_surf, i, vsp.XS_CIRCLE)
			xsec = vsp.GetXSec(hub_surf, i)
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Circle_Diameter'), d_hub )

	# --- Creating rotor blades --- #

	for i in range(int(n_lift_rotor/2)):
		rotor_id = vsp.AddGeom('PROP')
		vsp.SetParmVal(rotor_id, 'PropMode', 'Design', 1)
		vsp.SetParmVal(rotor_id, 'Diameter', 'Design', 2*r_lift_rotor)
		vsp.SetParmVal(rotor_id, 'NumBlade', 'Design', n_blade)

		vsp.SetParmVal( rotor_id, 	'X_Rel_Location', 	'XForm', 	X_Rel_Locations[i] 			   )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Location', 	'XForm', 	Y_Rel_Locations[i] 			   )
		vsp.SetParmVal( rotor_id, 	'Z_Rel_Location', 	'XForm', 	Z_Rel_Locations[i] - 0.5*l_hub )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Rotation', 	'XForm', 	90.0 			   			   )
		vsp.SetParmVal( rotor_id, 	'Sym_Planar_Flag', 	'Sym', 		2 				   			   )

def NASA_LPC_Propeller(n_propeller=1, n_blade=3, r_propeller=1.0, l_fuse=30.0):

	# --- Creating propeller hubs --- #

	# Parameters
	l_hub  = 1.0/30.0*l_fuse
	# d_hub  = 0.8*2.25/30.0*l_fuse
	d_hub  = 1.5/9.0*r_propeller

	X_Rel_Locations = [ l_fuse 			]
	Y_Rel_Locations = [ 0.0 			]
	Z_Rel_Locations = [ 0.118292*l_fuse ]

	for i in range(int(n_propeller)):
		hub_id 	= vsp.AddGeom('FUSELAGE')
		hub_surf 	= vsp.GetXSecSurf(hub_id, 0)
		xsec_num	= vsp.GetNumXSec(hub_surf)
		vsp.SetParmVal( hub_id, 'Length', 'Design', l_hub)
		vsp.SetParmVal( hub_id, 'X_Rel_Location', 	'XForm', 	X_Rel_Locations[i] )
		vsp.SetParmVal( hub_id, 'Y_Rel_Location', 	'XForm', 	Y_Rel_Locations[i] )
		vsp.SetParmVal( hub_id, 'Z_Rel_Location', 	'XForm', 	Z_Rel_Locations[i] )
		vsp.SetParmVal( hub_id, 'Y_Rel_Rotation', 	'XForm', 	0.0 			   )

		# Change shape of XSecs, and set height + width
		for i in range(1, xsec_num-1):
			vsp.ChangeXSecShape(hub_surf, i, vsp.XS_CIRCLE)
			xsec = vsp.GetXSec(hub_surf, i)
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Circle_Diameter'), d_hub )

	# --- Creating propeller blades --- #

	for i in range(int(n_propeller)):
		rotor_id = vsp.AddGeom('PROP')
		vsp.SetParmVal(rotor_id, 'PropMode', 'Design', 1)
		vsp.SetParmVal(rotor_id, 'Diameter', 'Design', 2*r_propeller)
		vsp.SetParmVal(rotor_id, 'NumBlade', 'Design', n_blade)

		vsp.SetParmVal( rotor_id, 	'X_Rel_Location', 	'XForm', 	X_Rel_Locations[i] + 0.5*l_hub )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Location', 	'XForm', 	Y_Rel_Locations[i] 			   )
		vsp.SetParmVal( rotor_id, 	'Z_Rel_Location', 	'XForm', 	Z_Rel_Locations[i] 			   )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Rotation', 	'XForm', 	0.0 			   			   )
		vsp.SetParmVal( rotor_id, 	'Sym_Planar_Flag', 	'Sym', 		2 				   			   )


	















	