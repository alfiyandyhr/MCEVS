import openvsp as vsp
import numpy as np

def NASA_LPC_Landing_Gear(l_strut=0.3048, d_fuse_max=1.8, l_fuse=30.0):

	# Nose, left, right
	X_Rel_Locs = [  0.121627,  0.418359,  0.418359 ]
	Y_Rel_Locs = [  0.000000, -0.900000,  0.900000 ]
	Z_Rel_Locs = [  0.025612,  0.043950,  0.043950 ]
	Height	   = [  5.000000,  6.127066,  6.127066 ]
	p_height   = [  1.000000,  0.980000,  0.980000 ]
	Width      = [  4.939956,  6.150259,  6.150259 ]
	X_Rel_Rots = [ -90.00000, -105.0000, -75.00000 ]
	Y_Rel_Rots = [  00.00000,  180.0000,  180.0000 ]
	Sweep 	   = [ -20.00000, -20.00000, -20.00000 ]
	
	lg_ids = [vsp.AddGeom('WING'), vsp.AddGeom('WING'), vsp.AddGeom('WING')]
	for i, lg_id in enumerate(lg_ids):
		vsp.SetParmVal( lg_id, 'X_Rel_Location', 	'XForm', 	X_Rel_Locs[i]*l_fuse	 	  									   )
		vsp.SetParmVal( lg_id, 'Y_Rel_Location', 	'XForm', 	Y_Rel_Locs[i]*0.5*0.8*Width[i]/6.150259*d_fuse_max				   )
		vsp.SetParmVal( lg_id, 'Z_Rel_Location', 	'XForm', 	Z_Rel_Locs[i]*l_fuse-0.5*p_height[i]*Height[i]/6.150259*d_fuse_max )
		
		vsp.SetParmVal( lg_id, 'X_Rel_Rotation', 	'XForm', 	X_Rel_Rots[i] 						 							   )
		vsp.SetParmVal( lg_id, 'Y_Rel_Rotation', 	'XForm', 	Y_Rel_Rots[i] 						 	  						   )
		vsp.SetParmVal( lg_id, 'Sym_Planar_Flag',	'Sym',		0 									 	  						   )

		vsp.SetParmValUpdate( lg_id, 'Span', 				'XSec_1', 	l_strut				 									   )
		vsp.SetParmValUpdate( lg_id, 'Root_Chord', 			'XSec_1', 	0.75/6.15*d_fuse_max 									   )
		vsp.SetParmValUpdate( lg_id, 'Tip_Chord', 			'XSec_1', 	0.75/6.15*d_fuse_max 									   )
		vsp.SetParmValUpdate( lg_id, 'Sweep', 				'XSec_1', 	Sweep[i] 	 		  	 								   )

		# Change airfoil shape using NACA 0050
		lg_surf = vsp.GetXSecSurf(lg_id, 0)
		xsec_num = vsp.GetNumXSec(lg_surf)
		for i in range(xsec_num):
			lg_xsec = vsp.GetXSec(lg_surf, i)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'ThickChord'), 0.5)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'Camber'), 0.0)
			vsp.SetParmVal(vsp.GetXSecParm(lg_xsec, 'CamberLoc'), 0.0)

	wheel_ids = [vsp.AddGeom('BODYOFREVOLUTION'), vsp.AddGeom('BODYOFREVOLUTION'), vsp.AddGeom('BODYOFREVOLUTION')]
	X_wheel   = [1.0, 2.0, 0.0]
	for i, wheel_id in enumerate(wheel_ids):
		vsp.SetParmVal( wheel_id, 'X_Rel_Location', 	'XForm', 	X_Rel_Locs[i]*l_fuse	 	  									   							 )
		vsp.SetParmVal( wheel_id, 'Y_Rel_Location', 	'XForm', 	Y_Rel_Locs[i]*0.5*0.8*Width[i]/6.150259*d_fuse_max - X_wheel[i]*0.5*0.75*1/30*l_fuse		 )
		vsp.SetParmVal( wheel_id, 'Z_Rel_Location', 	'XForm', 	Z_Rel_Locs[i]*l_fuse-0.5*p_height[i]*Height[i]/6.150259*d_fuse_max - l_strut - 0.5/30*l_fuse )

		vsp.SetParmVal( wheel_id, 'Z_Rel_Rotation',		'XForm',		90 				 )
		vsp.SetParmVal( wheel_id, 'Diameter', 			'Design',		1/30*l_fuse 	 )
		vsp.SetParmVal( wheel_id, 'Circle_Diameter', 	'XSecCurve', 	0.75*1/30*l_fuse )















