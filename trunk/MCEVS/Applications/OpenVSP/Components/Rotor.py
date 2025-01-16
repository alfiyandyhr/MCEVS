import MCEVS
import openvsp as vsp
import numpy as np

def NASA_QR_Lift_Rotor(n_lift_rotor=4, r_lift_rotor=9.159, n_blade=5, r_hub=None, l_fuse=21.0, d_fuse_max=6.745500, boom_ids=[None]):

	# --- Creating rotor hubs --- #

	# Baseline params
	# l_hub  			= 1.7/21.0*l_fuse
	# l_hub 			= 2*0.12*r_lift_rotor
	l_hub = r_hub
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
		vsp.SetParmVal( rotor_id, 	'PropMode', 		'Design', 	1 			   )
		vsp.SetParmVal( rotor_id, 	'Diameter', 		'Design', 	2*r_lift_rotor )
		vsp.SetParmVal( rotor_id, 	'NumBlade', 		'Design', 	n_blade 	   )
		vsp.SetParmVal( rotor_id, 	'UseBeta34Flag', 	'Design',	1   		   )
		vsp.SetParmVal( rotor_id, 	'Beta34', 			'Design',	0.0 		   )
		vsp.SetParmVal( rotor_id, 	'ConstructXoC',		 'Design',	0.25 		   )
		vsp.SetParmVal( rotor_id, 	'FeatherAxisXoC', 	'Design',	0.25 		   )

		vsp.SetParmVal( rotor_id, 	'X_Rel_Location', 	'XForm', 	l_hub/2 		   )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Location', 	'XForm', 	0.00 			   )
		vsp.SetParmVal( rotor_id, 	'Z_Rel_Location', 	'XForm', 	0.00 			   )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Rotation', 	'XForm', 	Y_Rel_Rotations[i] )
		vsp.SetParmVal( rotor_id, 	'Sym_Planar_Flag', 	'Sym', 		0    			   )

		vsp.SetParmVal( rotor_id, 	'Trans_Attach_Flag','Attach', 	1.0 )
		vsp.SetParmVal( rotor_id, 	'Rots_Attach_Flag', 'Attach', 	1.0 )

		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 		'XSec_0',	0.12 )
		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 		'XSec_1',	0.75 )
		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 		'XSec_2',	1.00 )

		# Change airfoil shape using BOEING-VERTOL VR-12 AIRFOIL
		airfoil_dir = MCEVS.__file__[:-11] + 'Applications/OpenVSP/Components/Airfoils/VR12.dat'
		rotor_surf = vsp.GetXSecSurf(rotor_id, 0)
		xsec_num = vsp.GetNumXSec(rotor_surf)
		for i in range(xsec_num):
			vsp.ChangeXSecShape(rotor_surf, i, vsp.XS_FILE_AIRFOIL)
			xsec = vsp.GetXSec(rotor_surf, i)
			vsp.ReadFileAirfoil(xsec, airfoil_dir)

		# Chord setting
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Chord', 	0        )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Chord', 	0.120000 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Chord', 	0.175000 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Chord', 	0.285000 )
		vsp.SetParmVal( rotor_id, 	'r_3',  			'Chord', 	0.395000 )
		vsp.SetParmVal( rotor_id, 	'r_4',  			'Chord', 	0.505000 )
		vsp.SetParmVal( rotor_id, 	'r_5',  			'Chord', 	0.615000 )
		vsp.SetParmVal( rotor_id, 	'r_6',  			'Chord', 	0.725000 )
		vsp.SetParmVal( rotor_id, 	'r_7',  			'Chord', 	0.835000 )
		vsp.SetParmVal( rotor_id, 	'r_8',  			'Chord', 	0.945000 )
		vsp.SetParmVal( rotor_id, 	'r_9',  			'Chord', 	1.000000 )
		vsp.SetParmVal( rotor_id, 	'crd_0', 			'Chord', 	0.072500 )
		vsp.SetParmVal( rotor_id, 	'crd_1', 			'Chord', 	0.071475 )
		vsp.SetParmVal( rotor_id, 	'crd_2', 			'Chord', 	0.069425 )
		vsp.SetParmVal( rotor_id, 	'crd_3', 			'Chord', 	0.067375 )
		vsp.SetParmVal( rotor_id, 	'crd_4', 			'Chord', 	0.065325 )
		vsp.SetParmVal( rotor_id, 	'crd_5', 			'Chord', 	0.063275 )
		vsp.SetParmVal( rotor_id, 	'crd_6', 			'Chord', 	0.061225 )
		vsp.SetParmVal( rotor_id, 	'crd_7', 			'Chord', 	0.059175 )
		vsp.SetParmVal( rotor_id, 	'crd_8', 			'Chord', 	0.057125 )
		vsp.SetParmVal( rotor_id, 	'crd_9', 			'Chord', 	0.056100 )

		# Twist setting
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Twist', 	0     )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Twist', 	0.120 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Twist', 	0.750 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Twist', 	1.000 )
		vsp.SetParmVal( rotor_id, 	'tw_0', 			'Twist', 	7.560 )
		vsp.SetParmVal( rotor_id, 	'tw_1', 			'Twist', 	0.000 )
		vsp.SetParmVal( rotor_id, 	'tw_2', 			'Twist',   -3.000 )

		# Thickness-to-chord ratio setting (to have the Boeing Vertol VR-12 t/c)
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Thick', 	0       )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Thick', 	0.12000 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Thick', 	0.17500 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Thick', 	0.28500 )
		vsp.SetParmVal( rotor_id, 	'r_3',  			'Thick', 	0.39500 )
		vsp.SetParmVal( rotor_id, 	'r_4',  			'Thick', 	0.50500 )
		vsp.SetParmVal( rotor_id, 	'r_5',  			'Thick', 	0.61500 )
		vsp.SetParmVal( rotor_id, 	'r_6',  			'Thick', 	0.72500 )
		vsp.SetParmVal( rotor_id, 	'r_7',  			'Thick', 	0.83500 )
		vsp.SetParmVal( rotor_id, 	'r_8',  			'Thick', 	0.94500 )
		vsp.SetParmVal( rotor_id, 	'r_9',  			'Thick', 	1.00000 )
		vsp.SetParmVal( rotor_id, 	'toc_0',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_1',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_2',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_3',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_4',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_5',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_6',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_7',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_8',  			'Thick', 	0.10637 )
		vsp.SetParmVal( rotor_id, 	'toc_9',  			'Thick', 	0.10637 )

def NASA_LPC_Lift_Rotor(l_hub:float, d_hub:float, n_lift_rotor=8, n_blade=2, r_lift_rotor=5.0, l_fuse=30.0, wing_AR=12.12761, wing_S=210.27814, boom_ids=None):

	# --- Creating rotor hubs --- #

	# Parameters
	# l_hub  = 1.0/30.0*l_fuse
	# d_hub  = 0.8*2.25/30.0*l_fuse
	# d_hub  = 2.25/10.0*r_lift_rotor
	l_boom  = 0.8*8.0/30.0*l_fuse/1.6*r_lift_rotor if r_lift_rotor>=1.6 else 0.8*8.0/30.0*l_fuse
	d_boom = 1.0/30.0*l_fuse
	b = np.sqrt(wing_S * wing_AR)	# wing span
	wing_rc = 2.0 * ((5.4432819 + 4.1308305)/2 * 14.745022 / 210.27814 * wing_S) / (14.745022/25.26*b/2) / (1 + 4.1308305/5.4432819)
	wing_c_loc = 2.0 * ((4.130830 + 3.304664)/2 * 6.626930 / 210.27814 * wing_S) / (6.6269300/25.26*b/2) / (1 + 3.304664/4.130830)
	t_per_c = 0.17 # LS-417 airfoil

	wing_X0 = (8.889-1.889)/30*l_fuse
	wing_Z0 = (8.499-4.249)/30*l_fuse

	if n_lift_rotor == 4:
		U_Attach_Locs 	= [ [0.05, 0.95] ]
		V_Attach_Locs 	= [ [0.75, 0.75] ]

	if n_lift_rotor == 8:
		U_Attach_Locs 	= [ [0.05, 0.95], [0.05, 0.95] ]
		V_Attach_Locs 	= [ [0.75, 0.75], [0.75, 0.75] ]

	hub_ids = []
	for i in range(int(n_lift_rotor/4)):
		hub_id1 	= vsp.AddGeom('FUSELAGE', boom_ids[i])
		hub_id2 	= vsp.AddGeom('FUSELAGE', boom_ids[i])
		hub_ids.append(hub_id1)
		hub_ids.append(hub_id2)
		hub_surf1 	= vsp.GetXSecSurf(hub_id1, 0)
		hub_surf2 	= vsp.GetXSecSurf(hub_id2, 0)
		xsec_num1	= vsp.GetNumXSec(hub_surf1)
		xsec_num2	= vsp.GetNumXSec(hub_surf2)
		for j in range(2):
			hub_id = hub_id1 if j==0 else hub_id2
			vsp.SetParmVal( hub_id, 'Length', 'Design', l_hub)
			vsp.SetParmVal( hub_id, 'X_Rel_Location', 		'XForm', 	0.0 				)
			vsp.SetParmVal( hub_id, 'Y_Rel_Location', 		'XForm', 	0.0 				)
			vsp.SetParmVal( hub_id, 'Z_Rel_Location', 		'XForm', 	0.1 				)
			vsp.SetParmVal( hub_id, 'X_Rel_Rotation', 		'XForm', 	180.0 			    )
			vsp.SetParmVal( hub_id, 'Y_Rel_Rotation', 		'XForm', 	90.0 			    )
			vsp.SetParmVal( hub_id, 'Sym_Ancestor', 		'Sym', 		0 				   	)
			vsp.SetParmVal( hub_id, 'Sym_Planar_Flag', 		'Sym', 		2 				    )
			vsp.SetParmVal( hub_id, 'Trans_Attach_Flag', 	'Attach',	2.0  			    )
			vsp.SetParmVal( hub_id, 'Rots_Attach_Flag',  	'Attach',	1.0  			    )
			vsp.SetParmVal( hub_id, 'U_Attach_Location', 	'Attach',	U_Attach_Locs[i][j] )
			vsp.SetParmVal( hub_id, 'V_Attach_Location', 	'Attach',	V_Attach_Locs[i][j] )

		# Change shape of XSecs, and set height + width
		for i in range(1, xsec_num1-1):
			vsp.ChangeXSecShape(hub_surf1, i, vsp.XS_CIRCLE)
			xsec = vsp.GetXSec(hub_surf1, i)
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Circle_Diameter'), d_hub )

		# Change shape of XSecs, and set height + width
		for i in range(1, xsec_num2-1):
			vsp.ChangeXSecShape(hub_surf2, i, vsp.XS_CIRCLE)
			xsec = vsp.GetXSec(hub_surf2, i)
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Circle_Diameter'), d_hub )

	# --- Creating rotor blades --- #
	rotor_ids = []
	for i in range(int(n_lift_rotor/2)):
		rotor_id = vsp.AddGeom('PROP', hub_ids[i])
		rotor_ids.append(rotor_id)
		vsp.SetParmVal( rotor_id, 	'PropMode', 			'Design', 	1 			   )
		vsp.SetParmVal( rotor_id, 	'Diameter', 			'Design', 	2*r_lift_rotor )
		vsp.SetParmVal( rotor_id, 	'NumBlade', 			'Design', 	n_blade 	   )
		vsp.SetParmVal( rotor_id, 	'UseBeta34Flag', 		'Design',	1   		   )
		vsp.SetParmVal( rotor_id, 	'Beta34', 				'Design',	0.0 		   )
		vsp.SetParmVal( rotor_id, 	'Rotate', 				'Design',	90.0 		   )
		vsp.SetParmVal( rotor_id, 	'ConstructXoC',		 	'Design',	0.25 		   )
		vsp.SetParmVal( rotor_id, 	'FeatherAxisXoC', 		'Design',	0.25 		   )

		vsp.SetParmVal( rotor_id, 	'X_Rel_Location', 		'XForm', 	l_hub/2)
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Location', 		'XForm',    0.0 )
		vsp.SetParmVal( rotor_id, 	'Z_Rel_Location', 		'XForm', 	0.0    )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Rotation', 		'XForm', 	180.0  )
		vsp.SetParmVal( rotor_id, 	'Sym_Ancestor', 		'Sym', 		0      )
		vsp.SetParmVal( rotor_id, 	'Sym_Planar_Flag', 		'Sym', 		2 	   )
		vsp.SetParmVal( rotor_id, 	'Trans_Attach_Flag', 	'Attach',	2.0    )
		vsp.SetParmVal( rotor_id, 	'Rots_Attach_Flag',  	'Attach',	1.0    )
		vsp.SetParmVal( rotor_id, 	'U_Attach_Location', 	'Attach',	0.0    )
		vsp.SetParmVal( rotor_id, 	'V_Attach_Location', 	'Attach',	0.0    )

		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 			'XSec_0',	0.21 )
		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 			'XSec_1',	0.75 )
		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 			'XSec_2',	1.00 )

		# Change airfoil shape using BOEING-VERTOL VR-12 AIRFOIL
		airfoil_dir = MCEVS.__file__[:-11] + 'Applications/OpenVSP/Components/Airfoils/VR12.dat'
		rotor_surf = vsp.GetXSecSurf(rotor_id, 0)
		xsec_num = vsp.GetNumXSec(rotor_surf)
		for i in range(xsec_num):
			vsp.ChangeXSecShape(rotor_surf, i, vsp.XS_FILE_AIRFOIL)
			xsec = vsp.GetXSec(rotor_surf, i)
			vsp.ReadFileAirfoil(xsec, airfoil_dir)

		# Chord setting
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Chord', 	0        )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Chord', 	0.210000 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Chord', 	0.259375 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Chord', 	0.358125 )
		vsp.SetParmVal( rotor_id, 	'r_3',  			'Chord', 	0.456875 )
		vsp.SetParmVal( rotor_id, 	'r_4',  			'Chord', 	0.555625 )
		vsp.SetParmVal( rotor_id, 	'r_5',  			'Chord', 	0.654375 )
		vsp.SetParmVal( rotor_id, 	'r_6',  			'Chord', 	0.753125 )
		vsp.SetParmVal( rotor_id, 	'r_7',  			'Chord', 	0.851875 )
		vsp.SetParmVal( rotor_id, 	'r_8',  			'Chord', 	0.950625 )
		vsp.SetParmVal( rotor_id, 	'r_9',  			'Chord', 	1.000000 )
		vsp.SetParmVal( rotor_id, 	'crd_0', 			'Chord', 	0.557000 )
		vsp.SetParmVal( rotor_id, 	'crd_1', 			'Chord', 	0.548288 )
		vsp.SetParmVal( rotor_id, 	'crd_2', 			'Chord', 	0.530863 )
		vsp.SetParmVal( rotor_id, 	'crd_3', 			'Chord', 	0.513438 )
		vsp.SetParmVal( rotor_id, 	'crd_4', 			'Chord', 	0.496013 )
		vsp.SetParmVal( rotor_id, 	'crd_5', 			'Chord', 	0.478588 )
		vsp.SetParmVal( rotor_id, 	'crd_6', 			'Chord', 	0.461163 )
		vsp.SetParmVal( rotor_id, 	'crd_7', 			'Chord', 	0.443738 )
		vsp.SetParmVal( rotor_id, 	'crd_8', 			'Chord', 	0.426313 )
		vsp.SetParmVal( rotor_id, 	'crd_9', 			'Chord', 	0.417600 )

		# Twist setting
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Twist', 	0     )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Twist', 	0.210 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Twist', 	0.750 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Twist', 	1.000 )
		vsp.SetParmVal( rotor_id, 	'tw_0', 			'Twist', 	8.100 )
		vsp.SetParmVal( rotor_id, 	'tw_1', 			'Twist', 	0.000 )
		vsp.SetParmVal( rotor_id, 	'tw_2', 			'Twist',   -3.750 )

		# Thickness-to-chord ratio setting (to have the Boeing Vertol VR-12 t/c)
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Thick', 	0       )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Thick', 	0.210000 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Thick', 	0.259375 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Thick', 	0.358125 )
		vsp.SetParmVal( rotor_id, 	'r_3',  			'Thick', 	0.456875 )
		vsp.SetParmVal( rotor_id, 	'r_4',  			'Thick', 	0.555625 )
		vsp.SetParmVal( rotor_id, 	'r_5',  			'Thick', 	0.654375 )
		vsp.SetParmVal( rotor_id, 	'r_6',  			'Thick', 	0.753125 )
		vsp.SetParmVal( rotor_id, 	'r_7',  			'Thick', 	0.851875 )
		vsp.SetParmVal( rotor_id, 	'r_8',  			'Thick', 	0.950625 )
		vsp.SetParmVal( rotor_id, 	'r_9',  			'Thick', 	1.000000 )
		vsp.SetParmVal( rotor_id, 	'toc_0',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_1',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_2',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_3',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_4',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_5',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_6',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_7',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_8',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_9',  			'Thick', 	0.106370 )

	return hub_ids, rotor_ids

def NASA_LPC_Propeller(l_hub:float, d_hub:float, n_propeller=1, n_blade=3, r_propeller=1.0, l_fuse=30.0, fuse_id=None):

	# --- Creating propeller hubs --- #

	# Parameters
	# l_hub  = 1.0/30.0*l_fuse
	# d_hub  = 0.8*2.25/30.0*l_fuse
	# d_hub  = 1.5/9.0*r_propeller

	X_Rel_Locations = [ l_fuse 			]
	Y_Rel_Locations = [ 0.0 			]
	Z_Rel_Locations = [ 0.118292*l_fuse ]

	hub_ids = []
	for i in range(int(n_propeller)):
		hub_id 	= vsp.AddGeom('FUSELAGE', fuse_id)
		hub_ids.append(hub_id)
		hub_surf 	= vsp.GetXSecSurf(hub_id, 0)
		xsec_num	= vsp.GetNumXSec(hub_surf)
		vsp.SetParmVal( hub_id, 	'Length', 				'Design', 	l_hub )
		vsp.SetParmVal( hub_id, 	'X_Rel_Location', 		'XForm', 	0.0   )
		vsp.SetParmVal( hub_id, 	'Y_Rel_Location', 		'XForm', 	0.0   )
		vsp.SetParmVal( hub_id, 	'Z_Rel_Location', 		'XForm', 	0.0   )
		vsp.SetParmVal( hub_id, 	'Y_Rel_Rotation', 		'XForm', 	0.0   )
		vsp.SetParmVal( hub_id, 	'Sym_Planar_Flag', 		'Sym', 		0 	  )
		vsp.SetParmVal( hub_id, 	'Trans_Attach_Flag', 	'Attach',	2.0   )
		vsp.SetParmVal( hub_id, 	'Rots_Attach_Flag',  	'Attach',	1.0   )
		vsp.SetParmVal( hub_id, 	'U_Attach_Location', 	'Attach',	1.0   )
		vsp.SetParmVal( hub_id, 	'V_Attach_Location', 	'Attach',	0.0   )

		# Change shape of XSecs, and set height + width
		for i in range(1, xsec_num-1):
			vsp.ChangeXSecShape(hub_surf, i, vsp.XS_CIRCLE)
			xsec = vsp.GetXSec(hub_surf, i)
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Circle_Diameter'), d_hub )

	# --- Creating propeller blades --- #
	rotor_ids = []
	for i in range(int(n_propeller)):
		rotor_id = vsp.AddGeom('PROP', hub_ids[i])
		rotor_ids.append(rotor_id)
		vsp.SetParmVal( rotor_id, 	'PropMode', 			'Design', 	1 			  )
		vsp.SetParmVal( rotor_id, 	'Diameter', 			'Design', 	2*r_propeller )
		vsp.SetParmVal( rotor_id, 	'NumBlade', 			'Design', 	n_blade 	  )
		vsp.SetParmVal( rotor_id, 	'UseBeta34Flag', 		'Design',	1   		  )
		vsp.SetParmVal( rotor_id, 	'Beta34', 				'Design',	0.0 		  )
		vsp.SetParmVal( rotor_id, 	'ConstructXoC',		 	'Design',	0.25 		  )
		vsp.SetParmVal( rotor_id, 	'FeatherAxisXoC', 		'Design',	0.25 		  )

		vsp.SetParmVal( rotor_id, 	'X_Rel_Location', 		'XForm', 	l_hub/2)
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Location', 		'XForm',    0.0    )
		vsp.SetParmVal( rotor_id, 	'Z_Rel_Location', 		'XForm', 	0.0    )
		vsp.SetParmVal( rotor_id, 	'Y_Rel_Rotation', 		'XForm', 	180.0  )
		vsp.SetParmVal( rotor_id, 	'Sym_Planar_Flag', 		'Sym', 		0 	   )
		vsp.SetParmVal( rotor_id, 	'Trans_Attach_Flag', 	'Attach',	2.0    )
		vsp.SetParmVal( rotor_id, 	'Rots_Attach_Flag',  	'Attach',	1.0    )
		vsp.SetParmVal( rotor_id, 	'U_Attach_Location', 	'Attach',	0.0    )
		vsp.SetParmVal( rotor_id, 	'V_Attach_Location', 	'Attach',	0.0    )

		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 			'XSec_0',	0.21 )
		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 			'XSec_1',	0.75 )
		vsp.SetParmVal( rotor_id, 	'RadiusFrac', 			'XSec_2',	1.00 )

		# Change airfoil shape using BOEING-VERTOL VR-12 AIRFOIL
		airfoil_dir = MCEVS.__file__[:-11] + 'Applications/OpenVSP/Components/Airfoils/VR12.dat'
		rotor_surf = vsp.GetXSecSurf(rotor_id, 0)
		xsec_num = vsp.GetNumXSec(rotor_surf)
		for i in range(xsec_num):
			vsp.ChangeXSecShape(rotor_surf, i, vsp.XS_FILE_AIRFOIL)
			xsec = vsp.GetXSec(rotor_surf, i)
			vsp.ReadFileAirfoil(xsec, airfoil_dir)

		# Chord setting
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Chord', 	0        )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Chord', 	0.210000 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Chord', 	0.259375 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Chord', 	0.358125 )
		vsp.SetParmVal( rotor_id, 	'r_3',  			'Chord', 	0.456875 )
		vsp.SetParmVal( rotor_id, 	'r_4',  			'Chord', 	0.555625 )
		vsp.SetParmVal( rotor_id, 	'r_5',  			'Chord', 	0.654375 )
		vsp.SetParmVal( rotor_id, 	'r_6',  			'Chord', 	0.753125 )
		vsp.SetParmVal( rotor_id, 	'r_7',  			'Chord', 	0.851875 )
		vsp.SetParmVal( rotor_id, 	'r_8',  			'Chord', 	0.950625 )
		vsp.SetParmVal( rotor_id, 	'r_9',  			'Chord', 	1.000000 )
		vsp.SetParmVal( rotor_id, 	'crd_0', 			'Chord', 	0.508600 )
		vsp.SetParmVal( rotor_id, 	'crd_1', 			'Chord', 	0.500656 )
		vsp.SetParmVal( rotor_id, 	'crd_2', 			'Chord', 	0.484769 )
		vsp.SetParmVal( rotor_id, 	'crd_3', 			'Chord', 	0.468881 )
		vsp.SetParmVal( rotor_id, 	'crd_4', 			'Chord', 	0.452994 )
		vsp.SetParmVal( rotor_id, 	'crd_5', 			'Chord', 	0.437106 )
		vsp.SetParmVal( rotor_id, 	'crd_6', 			'Chord', 	0.421219 )
		vsp.SetParmVal( rotor_id, 	'crd_7', 			'Chord', 	0.405331 )
		vsp.SetParmVal( rotor_id, 	'crd_8', 			'Chord', 	0.389444 )
		vsp.SetParmVal( rotor_id, 	'crd_9', 			'Chord', 	0.381500 )

		# Twist setting
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Twist', 	0     )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Twist', 	0.210 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Twist', 	0.750 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Twist', 	1.000 )
		vsp.SetParmVal( rotor_id, 	'tw_0', 			'Twist', 	19.32 )
		vsp.SetParmVal( rotor_id, 	'tw_1', 			'Twist', 	0.000 )
		vsp.SetParmVal( rotor_id, 	'tw_2', 			'Twist',   -8.330 )

		# Thickness-to-chord ratio setting (to have the Boeing Vertol VR-12 t/c)
		vsp.SetParmVal( rotor_id, 	'CrvType', 			'Thick', 	0       )
		vsp.SetParmVal( rotor_id, 	'r_0',  			'Thick', 	0.210000 )
		vsp.SetParmVal( rotor_id, 	'r_1',  			'Thick', 	0.259375 )
		vsp.SetParmVal( rotor_id, 	'r_2',  			'Thick', 	0.358125 )
		vsp.SetParmVal( rotor_id, 	'r_3',  			'Thick', 	0.456875 )
		vsp.SetParmVal( rotor_id, 	'r_4',  			'Thick', 	0.555625 )
		vsp.SetParmVal( rotor_id, 	'r_5',  			'Thick', 	0.654375 )
		vsp.SetParmVal( rotor_id, 	'r_6',  			'Thick', 	0.753125 )
		vsp.SetParmVal( rotor_id, 	'r_7',  			'Thick', 	0.851875 )
		vsp.SetParmVal( rotor_id, 	'r_8',  			'Thick', 	0.950625 )
		vsp.SetParmVal( rotor_id, 	'r_9',  			'Thick', 	1.000000 )
		vsp.SetParmVal( rotor_id, 	'toc_0',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_1',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_2',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_3',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_4',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_5',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_6',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_7',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_8',  			'Thick', 	0.106370 )
		vsp.SetParmVal( rotor_id, 	'toc_9',  			'Thick', 	0.106370 )

	return hub_ids, rotor_ids
