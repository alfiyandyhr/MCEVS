import openvsp as vsp
import numpy as np

def NASA_QR_Boom(n_lift_rotor=4, r_lift_rotor=9.159, l_fuse=21.0, d_fuse_max=6.745500, fuse_id=None):
	
	X_Rot 			= [   0.0000000,  180.0000, 0.000000,  180.0000 ]
	Span  			= [ [ 6.4655212,  5.974479 ], [ 6.4655212,  5.974479 ], [ 2.4901480,  2.023700,  8.100000 ], [ 2.4901480,  2.023700,  8.100000 ] ]
	Root_Chord 		= [ [ 4.4800000,  2.800000 ], [ 4.4800000,  2.800000 ], [ 4.2666667,  3.266667,  2.770833 ], [ 4.2666667,  3.266667,  2.770833 ] ]
	Tip_Chord		= [ [ 2.8000000,  1.120000 ], [ 2.8000000,  1.120000 ], [ 3.2666667,  2.770833,  1.066667 ], [ 3.2666667,  2.770833,  1.066667 ] ]
	Sweep 			= [ [-45.000000, -45.00000 ], [-45.000000, -45.00000 ], [ 46.685950,  46.68595,  40.00000 ], [ 46.685950,  46.68595,  40.00000 ] ]
	Sweep_Location	= [ [ 0.0000000,  0.000000 ], [ 0.0000000,  0.000000 ], [ 0.0000000,  0.000000,  0.000000 ], [ 0.0000000,  0.000000,  0.000000 ] ]
	Sec_SW_Location = [ [ 1.0000000,  1.000000 ], [ 1.0000000,  1.000000 ], [ 1.0000000,  1.000000,  1.000000 ], [ 1.0000000,  1.000000,  1.000000 ] ]
	Twist 			= [ [ 3.0000000,  3.000000 ], [ 3.0000000,  3.000000 ], [ 0.0000000, -1.000000, -1.000000 ], [ 0.0000000, -1.000000, -1.000000 ] ]
	Twist_Location	= [ [ 0.2500000,  0.250000 ], [ 0.2500000,  0.250000 ], [ 0.2500000,  0.250000,  0.250000 ], [ 0.2500000,  0.250000,  0.250000 ] ]
	Dihedral 		= [ [ 8.0000000,  8.000000 ], [-8.0000000, -8.000000 ], [ 11.500000,  17.00000,  21.00000 ], [-11.500000, -17.00000, -21.00000 ] ]
	SectTess_U 		= [ [ 2.0000000,  2.000000 ], [ 2.0000000,  2.000000 ], [ 2.0000000,  2.000000,  2.000000 ], [ 2.0000000,  2.000000,  2.000000 ] ]

	boom_ids = []
	for i in range(int(n_lift_rotor)):
		boom_id 	= vsp.AddGeom('WING', fuse_id)
		boom_ids.append(boom_id)
		vsp.InsertXSec(boom_id, 1, vsp.XS_FOUR_SERIES)
		if i in [2,3]:
			vsp.InsertXSec(boom_id, 1, vsp.XS_FOUR_SERIES)
		boom_surf 	= vsp.GetXSecSurf(boom_id, 0)
		xsec_num 	= vsp.GetNumXSec(boom_surf)

		for j in range(len(Span[i])):
			# Span[i][j] 			= Span[i][j]/21.0*l_fuse/2.5*r_lift_rotor
			Span[i][j] 			= Span[i][j]/21.0*l_fuse * 0.7
			Root_Chord[i][j] 	= Root_Chord[i][j]/21.0*l_fuse
			Tip_Chord[i][j] 	= Tip_Chord[i][j]/21.0*l_fuse

		vsp.SetParmVal( boom_id, 'X_Rel_Location', 		'XForm', 		0.0 	 )
		vsp.SetParmVal( boom_id, 'Y_Rel_Location', 		'XForm', 		0.0   	 )
		vsp.SetParmVal( boom_id, 'Z_Rel_Location', 		'XForm', 		0.0  	 )
		vsp.SetParmVal( boom_id, 'X_Rel_Rotation', 		'XForm', 		X_Rot[i] )
		vsp.SetParmVal( boom_id, 'Y_Rel_Rotation', 		'XForm', 		0.0 	 )
		vsp.SetParmVal( boom_id, 'Z_Rel_Rotation', 		'XForm', 		0.0 	 )
		vsp.SetParmVal( boom_id, 'Tess_W', 				'Shape', 		13 		 )
		vsp.SetParmVal( boom_id, 'LECluster', 			'WingGeom', 	1.0 	 )
		vsp.SetParmVal( boom_id, 'TECluster', 			'WingGeom', 	1.0 	 )
		vsp.SetParmVal( boom_id, 'Sym_Planar_Flag', 	'Sym', 			0 		 )
		vsp.SetParmVal( boom_id, 'Trans_Attach_Flag', 	'Attach',  		2.0 	 )
		vsp.SetParmVal( boom_id, 'Rots_Attach_Flag',  	'Attach',  		1.0 	 )
		vsp.SetParmVal( boom_id, 'U_Attach_Location', 	'Attach',  		0.4 	 )
		vsp.SetParmVal( boom_id, 'V_Attach_Location', 	'Attach',  		0.75 	 )

		for j in range(1,xsec_num):
			vsp.SetParmValUpdate( boom_id, 'Span', 					f'XSec_{j}', 	Span[i][j-1]			)
			vsp.SetParmValUpdate( boom_id, 'Root_Chord', 			f'XSec_{j}', 	Root_Chord[i][j-1] 		)
			vsp.SetParmValUpdate( boom_id, 'Tip_Chord', 			f'XSec_{j}', 	Tip_Chord[i][j-1] 		)
			vsp.SetParmValUpdate( boom_id, 'Sweep', 				f'XSec_{j}', 	Sweep[i][j-1] 		  	)
			vsp.SetParmValUpdate( boom_id, 'Sweep_Location',		f'XSec_{j}',	Sweep_Location[i][j-1] 	)
			vsp.SetParmValUpdate( boom_id, 'Sec_Sweep_Location',	f'XSec_{j}',	Sec_SW_Location[i][j-1]	)
			vsp.SetParmValUpdate( boom_id, 'Twist', 				f'XSec_{j}', 	Twist[i][j-1] 		  	)
			vsp.SetParmValUpdate( boom_id, 'Twist_Location',  		f'XSec_{j}',	Twist_Location[i][j-1] 	)
			vsp.SetParmValUpdate( boom_id, 'Dihedral', 				f'XSec_{j}', 	Dihedral[i][j-1] 	  	)
			vsp.SetParmValUpdate( boom_id, 'SectTess_U', 			f'XSec_{j}', 	SectTess_U[i][j-1] 		)

		# Change airfoil shape using NACA 4-series
		ThickCord 	= [[0.20, 0.20, 0.36], [0.20, 0.20, 0.36], [0.27273, 0.27273, 0.20000, 0.32000], [0.27273, 0.27273, 0.20000, 0.32000]]
		Camber 		= [[0.00, 0.00, 0.00], [0.00, 0.00, 0.00], [0.02700, 0.00000, 0.00000, 0.00000], [0.02700, 0.00000, 0.00000, 0.00000]]
		CamberLoc 	= [[0.00, 0.00, 0.00], [0.00, 0.00, 0.00], [0.24545, 0.20000, 0.20000, 0.20000], [0.24545, 0.20000, 0.20000, 0.20000]]
		Invert      = [[0, 0, 0], [1, 1, 1], [0, 0, 0, 0], [1, 1, 1, 1]]
		for j in range(xsec_num):
			boom_xsec = vsp.GetXSec(boom_surf, j)
			vsp.SetParmVal(vsp.GetXSecParm(boom_xsec, 'ThickChord'), 	ThickCord[i][j])
			vsp.SetParmVal(vsp.GetXSecParm(boom_xsec, 'Camber'), 		Camber[i][j])
			vsp.SetParmVal(vsp.GetXSecParm(boom_xsec, 'CamberLoc'), 	CamberLoc[i][j])
			vsp.SetParmVal(vsp.GetXSecParm(boom_xsec, 'Invert'), 		Invert[i][j])

	return boom_ids

def NASA_LPC_Boom(n_lift_rotor=4, r_lift_rotor=1.0, l_fuse=30.0, wing_AR=12.12761, wing_S=210.27814, wing_id=None):

	# Parameters
	l_boom  = 1.6*8.0/30.0*l_fuse/1.6*r_lift_rotor if r_lift_rotor>=1.6 else 1.6*8.0/30.0*l_fuse
	b = np.sqrt(wing_S * wing_AR)	# wing span
	wing_rc = 2.0 * ((5.4432819 + 4.1308305)/2 * 14.745022 / 210.27814 * wing_S) / (14.745022/25.26*b/2) / (1 + 4.1308305/5.4432819)
	wing_c_loc = 2.0 * ((4.130830 + 3.304664)/2 * 6.626930 / 210.27814 * wing_S) / (6.6269300/25.26*b/2) / (1 + 3.304664/4.130830)
	t_per_c = 0.17 # LS-417 airfoil

	wing_X0 = (8.889-1.889)/30*l_fuse
	wing_Z0 = (8.499-4.249)/30*l_fuse

	if n_lift_rotor == 4:
		X_Rel_Locations = [ -1.148 if r_lift_rotor<=1.6 else -1.500]
		Y_Rel_Locations = [  0.00  ]
		Z_Rel_Locations = [ -0.20  ]
		U_Attach_Locs	= [  0.31  ]
		V_Attach_Locs	= [  0.50  ]

	if n_lift_rotor == 8:
		X_Rel_Locations = [ -1.148, -1.300  ]
		Y_Rel_Locations = [  0.000,  0.000  ]
		Z_Rel_Locations = [ -0.200, -0.200  ]
		U_Attach_Locs	= [  0.250,  0.420  ]
		V_Attach_Locs	= [  0.500,  0.500  ]		

	boom_ids = []
	for i in range(int(n_lift_rotor/4)):
		boom_id 	= vsp.AddGeom('FUSELAGE', wing_id)
		boom_ids.append(boom_id)
		boom_surf 	= vsp.GetXSecSurf(boom_id, 0)
		xsec_num	= vsp.GetNumXSec(boom_surf)
		vsp.SetParmVal( boom_id, 'Length', 'Design', l_boom)
		vsp.SetParmVal( boom_id, 'X_Rel_Location', 		'XForm', 	X_Rel_Locations[i] )
		vsp.SetParmVal( boom_id, 'Y_Rel_Location', 		'XForm', 	Y_Rel_Locations[i] )
		vsp.SetParmVal( boom_id, 'Z_Rel_Location', 		'XForm', 	Z_Rel_Locations[i] )
		vsp.SetParmVal( boom_id, 'Sym_Ancestor', 		'Sym', 		0 				   )
		vsp.SetParmVal( boom_id, 'Sym_Planar_Flag', 	'Sym', 		2 				   )
		vsp.SetParmVal( boom_id, 'Trans_Attach_Flag', 	'Attach',	2.0  			   )
		vsp.SetParmVal( boom_id, 'Rots_Attach_Flag',  	'Attach',	1.0  			   )
		vsp.SetParmVal( boom_id, 'U_Attach_Location', 	'Attach',	U_Attach_Locs[i]   )
		vsp.SetParmVal( boom_id, 'V_Attach_Location', 	'Attach',	V_Attach_Locs[i]   )

		# Change shape of XSecs, and set height + width
		for i in range(1, xsec_num-1):
			vsp.ChangeXSecShape(boom_surf, i, vsp.XS_CIRCLE)
			xsec = vsp.GetXSec(boom_surf, i)
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Circle_Diameter'), 1.0/30.0*l_fuse )

	return boom_ids
	