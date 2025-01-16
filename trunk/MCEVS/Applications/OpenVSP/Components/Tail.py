import openvsp as vsp
import numpy as np

def NASA_LPC_Horizontal_Tail(area=19.75560, aspect_ratio=2.15182, l_fuse=30.0, fuse_id=None):

	# Baseline params
	span_base 	= 6.52000
	rc_base		= 3.78750
	tc_base 	= 2.27250
	x_pos_base 	= 27.4277
	y_pos_base  = 0.00000
	z_pos_base  = 8.00806
	tr_base 	= 0.60

	htail_id = vsp.AddGeom('WING', fuse_id)

	vsp.SetParmVal( htail_id, 'X_Rel_Location', 	'XForm', 		0.000  )
	vsp.SetParmVal( htail_id, 'Y_Rel_Location', 	'XForm', 		0.000  )
	vsp.SetParmVal( htail_id, 'Z_Rel_Location', 	'XForm', 	   -0.110  )
	vsp.SetParmVal( htail_id, 'Tess_W', 			'Shape', 		13 	   )
	vsp.SetParmVal( htail_id, 'LECluster', 			'WingGeom', 	1.0    )
	vsp.SetParmVal( htail_id, 'TECluster', 			'WingGeom', 	1.0    )
	vsp.SetParmVal( htail_id, 'Sym_Planar_Flag', 	'Sym', 			2 	   )
	vsp.SetParmVal( htail_id, 'Trans_Attach_Flag', 	'Attach',  		2.0    )
	vsp.SetParmVal( htail_id, 'Rots_Attach_Flag',  	'Attach',  		1.0    )
	vsp.SetParmVal( htail_id, 'U_Attach_Location', 	'Attach',  		0.8365 )
	vsp.SetParmVal( htail_id, 'V_Attach_Location', 	'Attach',  		0.7500 )

	# Calculating other params
	S 	= area
	AR 	= aspect_ratio
	b 	= np.sqrt(S * AR)/2
	rc 	= 2*S/b/(1+tr_base)/2
	tc 	= rc * tr_base

	vsp.SetParmValUpdate( htail_id, 'Span', 				'XSec_1', 	b 	 	   )
	vsp.SetParmValUpdate( htail_id, 'Root_Chord', 			'XSec_1', 	rc 	 	   )
	vsp.SetParmValUpdate( htail_id, 'Tip_Chord', 			'XSec_1', 	tc 	 	   )
	vsp.SetParmValUpdate( htail_id, 'Sweep', 				'XSec_1', 	0.00 	   )
	vsp.SetParmValUpdate( htail_id, 'Sweep_Location',		'XSec_1',	0.25 	   )
	vsp.SetParmValUpdate( htail_id, 'Sec_Sweep_Location',	'XSec_1',	1.00 	   )
	vsp.SetParmValUpdate( htail_id, 'Twist', 				'XSec_1', 	0.00 	   )
	vsp.SetParmValUpdate( htail_id, 'Twist_Location',  		'XSec_1',	0.25 	   )
	vsp.SetParmValUpdate( htail_id, 'Dihedral', 			'XSec_1', 	0.00 	   )

	vsp.SetParmValUpdate( htail_id, 'SectTess_U', 			'XSec_1', 	2 	 	   )

	vsp.SetParmValUpdate( htail_id, 'InLEMode',				'XSec_1',	0		   )
	vsp.SetParmValUpdate( htail_id, 'InLEStrength', 		'XSec_1',	1.0		   )
	vsp.SetParmValUpdate( htail_id, 'InLESweep', 			'XSec_1',	3.3246	   )
	vsp.SetParmValUpdate( htail_id, 'InLEDihedral', 		'XSec_1',	7.0285E-15 )

	vsp.SetParmValUpdate( htail_id, 'OutLEMode',			'XSec_1',	0		   )
	vsp.SetParmValUpdate( htail_id, 'OutLEStrength', 		'XSec_1',	1.0		   )
	vsp.SetParmValUpdate( htail_id, 'OutLESweep', 			'XSec_1',	3.3246	   )
	vsp.SetParmValUpdate( htail_id, 'OutLEDihedral', 		'XSec_1',  -7.0285E-15 )


	# Change airfoil shape using NACA 0015
	htail_xsec_surf = vsp.GetXSecSurf(htail_id, 0)
	for i in range(2):
		htail_xsec = vsp.GetXSec(htail_xsec_surf, i)
		vsp.SetParmVal(vsp.GetXSecParm(htail_xsec, 'ThickChord'), 0.15)
		vsp.SetParmVal(vsp.GetXSecParm(htail_xsec, 'Camber'), 0.0)
		vsp.SetParmVal(vsp.GetXSecParm(htail_xsec, 'CamberLoc'), 0.0)

	return htail_id

def NASA_LPC_Vertical_Tail(area=27.34325, aspect_ratio=1.17990, l_fuse=30.0, fuse_id=None):
	
	# Baseline params

	Span  			= [  1.1416544,  0.427290,  0.520148,  3.590907 ]
	Root_Chord 		= [  9.7371946,  5.948228,  5.264780,  4.784385 ]
	Tip_Chord		= [  5.9482276,  5.264780,  4.784385,  2.668020 ]
	Sweep 			= [  15.000000,  15.00000,  15.00000,  15.00000 ]
	Sweep_Location	= [  1.0000000,  1.000000,  1.000000,  1.000000 ]
	Sec_SW_Location	= [  0.6333330,  0.666667,  0.583333,  0.300000 ]
	Twist 			= [  0.0000000,  0.000000,  0.000000,  0.000000 ]
	Twist_Location	= [  0.2500000,  0.250000,  0.250000,  0.250000 ]
	Dihedral 		= [  0.0000000,  0.000000,  0.000000,  0.000000 ]

	SectTess_U 		= [  2.0000000,  2.000000,  3.000000,  4.000000 ]

	InLEMode		= [  0.0000000,  0.000000,  0.000000,  0.000000 ]
	InLEStrength	= [  1.0000000,  1.000000,  1.000000,  1.000000 ]
	InLESweep 		= [  74.421478,  61.83131,  49.99450,  40.60705 ]
	InLEDihedral 	= [  2.1284308,  2.135989,  2.695763,  3.264543 ]

	OutLEMode		= [  0.0000000,  0.000000,  0.000000,  0.000000 ]
	OutLEStrength	= [  1.0000000,  1.000000,  1.000000,  1.000000 ]
	OutLESweep 		= [  61.831313,  49.99450,  40.60705,  40.60705 ]
	OutLEDihedral 	= [ -1.469E-14, -1.09E-14, -9.15E-15, -9.15E-15 ]

	vtail_id = vsp.AddGeom('WING', fuse_id)
	vsp.InsertXSec(vtail_id, 1, vsp.XS_FOUR_SERIES)
	vsp.InsertXSec(vtail_id, 1, vsp.XS_FOUR_SERIES)
	vsp.InsertXSec(vtail_id, 1, vsp.XS_FOUR_SERIES)

	vsp.SetParmVal( vtail_id, 'X_Rel_Location', 	'XForm', 		0.000     )
	vsp.SetParmVal( vtail_id, 'Y_Rel_Location', 	'XForm', 		0.000     )
	vsp.SetParmVal( vtail_id, 'Z_Rel_Location', 	'XForm', 	   -0.07117   )
	vsp.SetParmVal( vtail_id, 'Tess_W', 			'Shape', 		13 		  )
	vsp.SetParmVal( vtail_id, 'X_Rel_Rotation', 	'XForm', 		90.0 	  )
	vsp.SetParmVal( vtail_id, 'Sym_Planar_Flag',	'Sym',			0 		  )
	vsp.SetParmVal( vtail_id, 'LECluster', 			'WingGeom', 	1.0 	  )
	vsp.SetParmVal( vtail_id, 'TECluster', 			'WingGeom', 	1.0 	  )
	vsp.SetParmVal( vtail_id, 'Sym_Planar_Flag', 	'Sym', 			0 	   	  )
	vsp.SetParmVal( vtail_id, 'Trans_Attach_Flag', 	'Attach',  		2.0    	  )
	vsp.SetParmVal( vtail_id, 'Rots_Attach_Flag',  	'Attach',  		1.0    	  )
	vsp.SetParmVal( vtail_id, 'U_Attach_Location', 	'Attach',  		0.7696485 )
	vsp.SetParmVal( vtail_id, 'V_Attach_Location', 	'Attach',  		0.7500000 )

	# Calculating other params
	S 	= area
	AR 	= aspect_ratio
	b 	= np.sqrt(S * AR)
	Span_i 			= []
	Root_Chord_i 	= []
	Tip_Chord_i 	= []

	for i in range(len(Span)):

		# Adjusting span
		span_i = Span[i]/5.68*b
		Span_i.append(span_i)

		# Adjusting root and tip chords
		area_i 	= (Root_Chord[i] + Tip_Chord[i])/2 * Span[i]
		S_i 	= area_i / 27.34325 * S
		tr_i 	= Tip_Chord[i] / Root_Chord[i]
		rc_i 	= 2.0 * S_i / span_i / (1 + tr_i)
		tc_i 	= tr_i * rc_i
		Root_Chord_i.append(rc_i)
		Tip_Chord_i.append(tc_i)

	for i in range(1,5):
		vsp.SetParmValUpdate( vtail_id, 'Span', 				f'XSec_{i}', 	Span_i[i-1]			)
		vsp.SetParmValUpdate( vtail_id, 'Root_Chord', 			f'XSec_{i}', 	Root_Chord_i[i-1] 	)
		vsp.SetParmValUpdate( vtail_id, 'Tip_Chord', 			f'XSec_{i}', 	Tip_Chord_i[i-1] 	)
		vsp.SetParmValUpdate( vtail_id, 'Sweep', 				f'XSec_{i}', 	Sweep[i-1] 		  	)
		vsp.SetParmValUpdate( vtail_id, 'Sec_Sweep_Location',	f'XSec_{i}',	Sec_SW_Location[i-1])
		vsp.SetParmValUpdate( vtail_id, 'Sweep_Location',		f'XSec_{i}',	Sweep_Location[i-1] )
		vsp.SetParmValUpdate( vtail_id, 'Twist', 				f'XSec_{i}', 	Twist[i-1] 		  	)
		vsp.SetParmValUpdate( vtail_id, 'Twist_Location',  		f'XSec_{i}',	Twist_Location[i-1] )
		vsp.SetParmValUpdate( vtail_id, 'Dihedral', 			f'XSec_{i}', 	Dihedral[i-1] 	  	)

		vsp.SetParmValUpdate( vtail_id, 'SectTess_U', 			f'XSec_{i}', 	SectTess_U[i-1] 	)

		vsp.SetParmValUpdate( vtail_id, 'InLEMode',				f'XSec_{i}',	InLEMode[i-1]		)
		vsp.SetParmValUpdate( vtail_id, 'InLEStrength', 		f'XSec_{i}',	InLEStrength[i-1]	)
		vsp.SetParmValUpdate( vtail_id, 'InLESweep', 			f'XSec_{i}',	InLESweep[i-1]		)
		vsp.SetParmValUpdate( vtail_id, 'InLEDihedral', 		f'XSec_{i}',	InLEDihedral[i-1]	)

		vsp.SetParmValUpdate( vtail_id, 'OutLEMode',			f'XSec_{i}',	OutLEMode[i-1]		)
		vsp.SetParmValUpdate( vtail_id, 'OutLEStrength', 		f'XSec_{i}',	OutLEStrength[i-1]	)
		vsp.SetParmValUpdate( vtail_id, 'OutLESweep', 			f'XSec_{i}',	OutLESweep[i-1]		)
		vsp.SetParmValUpdate( vtail_id, 'OutLEDihedral', 		f'XSec_{i}',	OutLEDihedral[i-1]	)


	# Change airfoil shape using NACA 0012, 0014, 0015, 0015, 0012
	series = [0.12, 0.14, 0.15, 0.15, 0.12]
	vtail_surf = vsp.GetXSecSurf(vtail_id, 0)
	xsec_num = vsp.GetNumXSec(vtail_surf)
	for i in range(xsec_num):
		vtail_xsec = vsp.GetXSec(vtail_surf, i)
		vsp.SetParmVal(vsp.GetXSecParm(vtail_xsec, 'ThickChord'), series[i])
		vsp.SetParmVal(vsp.GetXSecParm(vtail_xsec, 'Camber'), 0.0)
		vsp.SetParmVal(vsp.GetXSecParm(vtail_xsec, 'CamberLoc'), 0.0)

	return vtail_id





