import openvsp as vsp
import numpy as np

def NASA_QR_Landing_Gear(l_strut=0.3048, fuse_id=None):

	# Nose, left, right
	X_Rel_Locs = [  0.000000,  0.000000,  0.000000 ]
	Y_Rel_Locs = [  0.000000,  0.000000,  0.000000 ]
	Z_Rel_Locs = [ -0.030000, -0.100000, -0.100000 ]
	X_Rel_Rots = [ -90.00000, -105.0000, -75.00000 ]
	Y_Rel_Rots = [  00.00000,  180.0000,  180.0000 ]
	U_Attach_Location = [0.17, 0.60, 0.60]
	V_Attach_Location = [0.25, 0.32, 0.18]
	Sweep 	   = [ -20.00000, -20.00000, -20.00000 ]

	lg_ids = []
	for i in range(3):
		lg_id = vsp.AddGeom('WING', fuse_id)
		lg_ids.append(lg_id)
		vsp.SetParmVal( lg_id, 'X_Rel_Location', 	'XForm', 	X_Rel_Locs[i] 		 )
		vsp.SetParmVal( lg_id, 'Y_Rel_Location', 	'XForm', 	Y_Rel_Locs[i] 		 )
		vsp.SetParmVal( lg_id, 'Z_Rel_Location', 	'XForm', 	Z_Rel_Locs[i] 		 )
		
		vsp.SetParmVal( lg_id, 'X_Rel_Rotation', 	'XForm', 	X_Rel_Rots[i] 		 )
		vsp.SetParmVal( lg_id, 'Y_Rel_Rotation', 	'XForm', 	Y_Rel_Rots[i] 	 	 )
		vsp.SetParmVal( lg_id, 'Sym_Planar_Flag',	'Sym',		0 				 	 )
		vsp.SetParmVal( lg_id, 'Trans_Attach_Flag', 'Attach',  	2.0 	 			 )
		vsp.SetParmVal( lg_id, 'Rots_Attach_Flag',  'Attach',  	1.0 	 			 )
		vsp.SetParmVal( lg_id, 'U_Attach_Location', 'Attach',  	U_Attach_Location[i] )
		vsp.SetParmVal( lg_id, 'V_Attach_Location', 'Attach',  	V_Attach_Location[i] )

		vsp.SetParmValUpdate( lg_id, 'Span', 		'XSec_1', 	l_strut				 )
		vsp.SetParmValUpdate( lg_id, 'Root_Chord', 	'XSec_1', 	0.75/6.15*1.8 		 )
		vsp.SetParmValUpdate( lg_id, 'Tip_Chord', 	'XSec_1', 	0.75/6.15*1.8 		 )
		vsp.SetParmValUpdate( lg_id, 'Sweep', 		'XSec_1', 	Sweep[i] 	 		 )

		# Change airfoil shape using NACA 0050
		lg_surf = vsp.GetXSecSurf(lg_id, 0)
		xsec_num = vsp.GetNumXSec(lg_surf)
		for i in range(xsec_num):
			lg_xsec = vsp.GetXSec(lg_surf, i)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'ThickChord'), 0.5)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'Camber'), 0.0)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'CamberLoc'), 0.0)

	Y_Wheel 	 = [0.150,  0.180,  0.180]
	Z_Wheel 	 = [0.112,  0.080, -0.080]
	X_Wheel 	 = [0.000,  0.000,  0.000]
	U_Wheel 	 = [1.000,  1.000,  1.000]
	V_Wheel 	 = [0.250,  0.250,  0.250]
	X_Rots_Wheel = [0.000, -15.00, -165.0]
	Y_Rots_Wheel = [90.00,  90.00,  90.00]
	Z_Rots_Wheel = [0.000,  0.000,  0.000]
	wheel_ids = []
	for i in range(3):
		wheel_id = vsp.AddGeom('BODYOFREVOLUTION', lg_ids[i])
		wheel_ids.append(wheel_id)
		vsp.SetParmVal( wheel_id, 'X_Rel_Location', 	'XForm', 	X_Wheel[i] )
		vsp.SetParmVal( wheel_id, 'Y_Rel_Location', 	'XForm', 	Y_Wheel[i] )
		vsp.SetParmVal( wheel_id, 'Z_Rel_Location', 	'XForm', 	Z_Wheel[i] )

		vsp.SetParmVal( wheel_id, 'Trans_Attach_Flag', 	'Attach',  		2.0 	 )
		vsp.SetParmVal( wheel_id, 'Rots_Attach_Flag',  	'Attach',  		1.0 	 )
		vsp.SetParmVal( wheel_id, 'U_Attach_Location', 	'Attach',  		U_Wheel[i] )
		vsp.SetParmVal( wheel_id, 'V_Attach_Location', 	'Attach',  		V_Wheel[i] )
		vsp.SetParmVal( wheel_id, 'X_Rel_Rotation',		'XForm',		X_Rots_Wheel[i]  )
		vsp.SetParmVal( wheel_id, 'Y_Rel_Rotation',		'XForm',		Y_Rots_Wheel[i]  )
		vsp.SetParmVal( wheel_id, 'Z_Rel_Rotation',		'XForm',		Z_Rots_Wheel[i]	 )
		vsp.SetParmVal( wheel_id, 'Diameter', 			'Design',		1/30*9.0 	 )
		vsp.SetParmVal( wheel_id, 'Circle_Diameter', 	'XSecCurve', 	0.75*1/30*9.0 )

	return lg_ids, wheel_ids

def NASA_LPC_Landing_Gear(l_strut=0.3048, fuse_id=None):

	# Nose, left, right
	X_Rel_Locs = [  0.000000,  0.000000,  0.000000 ]
	Y_Rel_Locs = [  0.000000,  0.000000,  0.000000 ]
	Z_Rel_Locs = [ -0.030000, -0.030000, -0.030000 ]
	X_Rel_Rots = [ -90.00000, -105.0000, -75.00000 ]
	Y_Rel_Rots = [  00.00000,  180.0000,  180.0000 ]
	U_Attach_Location = [0.25, 0.50, 0.50]
	V_Attach_Location = [0.25, 0.35, 0.15]
	Sweep 	   = [ -20.00000, -20.00000, -20.00000 ]

	lg_ids = []
	for i in range(3):
		lg_id = vsp.AddGeom('WING', fuse_id)
		lg_ids.append(lg_id)
		vsp.SetParmVal( lg_id, 'X_Rel_Location', 	'XForm', 	X_Rel_Locs[i] 		 )
		vsp.SetParmVal( lg_id, 'Y_Rel_Location', 	'XForm', 	Y_Rel_Locs[i] 		 )
		vsp.SetParmVal( lg_id, 'Z_Rel_Location', 	'XForm', 	Z_Rel_Locs[i] 		 )
		
		vsp.SetParmVal( lg_id, 'X_Rel_Rotation', 	'XForm', 	X_Rel_Rots[i] 		 )
		vsp.SetParmVal( lg_id, 'Y_Rel_Rotation', 	'XForm', 	Y_Rel_Rots[i] 	 	 )
		vsp.SetParmVal( lg_id, 'Sym_Planar_Flag',	'Sym',		0 				 	 )
		vsp.SetParmVal( lg_id, 'Trans_Attach_Flag', 'Attach',  	2.0 	 			 )
		vsp.SetParmVal( lg_id, 'Rots_Attach_Flag',  'Attach',  	1.0 	 			 )
		vsp.SetParmVal( lg_id, 'U_Attach_Location', 'Attach',  	U_Attach_Location[i] )
		vsp.SetParmVal( lg_id, 'V_Attach_Location', 'Attach',  	V_Attach_Location[i] )

		vsp.SetParmValUpdate( lg_id, 'Span', 		'XSec_1', 	l_strut				 )
		vsp.SetParmValUpdate( lg_id, 'Root_Chord', 	'XSec_1', 	0.75/6.15*1.8 		 )
		vsp.SetParmValUpdate( lg_id, 'Tip_Chord', 	'XSec_1', 	0.75/6.15*1.8 		 )
		vsp.SetParmValUpdate( lg_id, 'Sweep', 		'XSec_1', 	Sweep[i] 	 		 )

		# Change airfoil shape using NACA 0050
		lg_surf = vsp.GetXSecSurf(lg_id, 0)
		xsec_num = vsp.GetNumXSec(lg_surf)
		for i in range(xsec_num):
			lg_xsec = vsp.GetXSec(lg_surf, i)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'ThickChord'), 0.5)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'Camber'), 0.0)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'CamberLoc'), 0.0)

	X_Wheel 	 = [0.000,  0.000,  0.000]
	Y_Wheel 	 = [0.150,  0.180,  0.180]
	Z_Wheel 	 = [0.112,  0.080, -0.080]
	U_Wheel 	 = [1.000,  1.000,  1.000]
	V_Wheel 	 = [0.250,  0.250,  0.250]
	X_Rots_Wheel = [0.000, -15.00, -165.0]
	Y_Rots_Wheel = [90.00,  90.00,  90.00]
	Z_Rots_Wheel = [0.000,  0.000,  0.000]
	wheel_ids = []
	for i in range(3):
		wheel_id = vsp.AddGeom('BODYOFREVOLUTION', lg_ids[i])
		wheel_ids.append(wheel_id)
		vsp.SetParmVal( wheel_id, 'X_Rel_Location', 	'XForm', 	X_Wheel[i] )
		vsp.SetParmVal( wheel_id, 'Y_Rel_Location', 	'XForm', 	Y_Wheel[i] )
		vsp.SetParmVal( wheel_id, 'Z_Rel_Location', 	'XForm', 	Z_Wheel[i] )

		vsp.SetParmVal( wheel_id, 'Trans_Attach_Flag', 	'Attach',  		2.0 	 )
		vsp.SetParmVal( wheel_id, 'Rots_Attach_Flag',  	'Attach',  		1.0 	 )
		vsp.SetParmVal( wheel_id, 'U_Attach_Location', 	'Attach',  		U_Wheel[i] )
		vsp.SetParmVal( wheel_id, 'V_Attach_Location', 	'Attach',  		V_Wheel[i] )
		vsp.SetParmVal( wheel_id, 'X_Rel_Rotation',		'XForm',		X_Rots_Wheel[i]  )
		vsp.SetParmVal( wheel_id, 'Y_Rel_Rotation',		'XForm',		Y_Rots_Wheel[i]  )
		vsp.SetParmVal( wheel_id, 'Z_Rel_Rotation',		'XForm',		Z_Rots_Wheel[i]	 )
		vsp.SetParmVal( wheel_id, 'Diameter', 			'Design',		1/30*9.0 	 )
		vsp.SetParmVal( wheel_id, 'Circle_Diameter', 	'XSecCurve', 	0.75*1/30*9.0 )

	return lg_ids, wheel_ids

