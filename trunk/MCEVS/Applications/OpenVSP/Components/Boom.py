import openvsp as vsp
import numpy as np

def NASA_LPC_Boom(n_lift_rotor=8, l_fuse=30.0, wing_AR=12.12761, wing_S=210.27814):

	# Parameters
	l_boom  = 0.8*8.0/30.0*l_fuse
	b = np.sqrt(wing_S * wing_AR)	# wing span
	wing_rc = 2.0 * ((5.4432819 + 4.1308305)/2 * 14.745022 / 210.27814 * wing_S) / (14.745022/25.26*b/2) / (1 + 4.1308305/5.4432819)
	wing_c_loc = 2.0 * ((4.130830 + 3.304664)/2 * 6.626930 / 210.27814 * wing_S) / (6.6269300/25.26*b/2) / (1 + 3.304664/4.130830)
	t_per_c = 0.17 # LS-417 airfoil

	wing_X0 = (8.889-1.889)/30*l_fuse
	wing_Z0 = (8.499-4.249)/30*l_fuse

	if n_lift_rotor == 4:
		X_Rel_Locations = [ wing_X0 - (l_boom - 0.5*wing_rc),
							wing_X0 + 0.5*wing_rc ]
		Y_Rel_Locations = [ 0.25*b, 0.25*b ]
		Z_Rel_Locations = (wing_Z0 - (t_per_c * wing_c_loc)) * np.ones(2)		

	if n_lift_rotor == 8:
		X_Rel_Locations = [ wing_X0 - (l_boom - 0.5*wing_rc),
							wing_X0 + 0.5*wing_rc,
							wing_X0 - (l_boom - 0.5*wing_rc),
							wing_X0 + 0.5*wing_rc ]
		Y_Rel_Locations = [ 14.745022/2/50.52*b,
							14.745022/2/50.52*b,
							(14.745022+6.626930/2)/50.52*b,
							(14.745022+6.626930/2)/50.52*b ]
		Z_Rel_Locations = (wing_Z0 - (t_per_c * wing_c_loc)) * np.ones(4)

	for i in range(int(n_lift_rotor/2)):
		boom_id 	= vsp.AddGeom('FUSELAGE')
		boom_surf 	= vsp.GetXSecSurf(boom_id, 0)
		xsec_num	= vsp.GetNumXSec(boom_surf)
		vsp.SetParmVal(boom_id, 'Length', 'Design', l_boom)
		vsp.SetParmVal(boom_id, 'X_Rel_Location', 'XForm', X_Rel_Locations[i])
		vsp.SetParmVal(boom_id, 'Y_Rel_Location', 'XForm', Y_Rel_Locations[i])
		vsp.SetParmVal(boom_id, 'Z_Rel_Location', 'XForm', Z_Rel_Locations[i])
		vsp.SetParmVal(boom_id, 'Sym_Planar_Flag', 'Sym', 2)

		# Change shape of XSecs, and set height + width
		for i in range(1, xsec_num-1):
			vsp.ChangeXSecShape(boom_surf, i, vsp.XS_CIRCLE)
			xsec = vsp.GetXSec(boom_surf, i)
			vsp.SetParmVal(	vsp.GetXSecParm(xsec, 'Circle_Diameter'), 1.0/30.0*l_fuse )
	