import openvsp as vsp
from MCEVS.Applications.OpenVSP.Components.Payload import Human
from MCEVS.Applications.OpenVSP.Components.Fuselage import NASA_QR_Fuselage
from MCEVS.Applications.OpenVSP.Components.Boom import NASA_QR_Boom
from MCEVS.Applications.OpenVSP.Components.Rotor import NASA_QR_Lift_Rotor
from MCEVS.Applications.OpenVSP.Components.Landing_Gear import NASA_QR_Landing_Gear

# n_pax = 4

# Human(N_PAX=n_pax, config='Multirotor')

# if n_pax == 1:
# 	l_fuse 		= 2.074	 # m
# 	d_fuse_max 	= 1.4 	 # m
# elif n_pax == 4:
# 	l_fuse 		= 5.2	 # m
# 	d_fuse_max 	= 1.8 	 # m

# # # NASA_QR_Fuselage()
# NASA_QR_Fuselage(l_fuse=l_fuse, d_fuse_max=d_fuse_max)
# NASA_QR_Boom(n_lift_rotor=4, l_fuse=l_fuse, d_fuse_max=d_fuse_max)
# NASA_QR_Lift_Rotor(n_lift_rotor=4, r_lift_rotor=2.0, n_blade=5, l_fuse=l_fuse, d_fuse_max=d_fuse_max)
# NASA_QR_Landing_Gear(l_strut=0.3, l_fuse=l_fuse, d_fuse_max=d_fuse_max)
# vsp.WriteVSPFile('vehicle.vsp3')

vsp.ReadVSPFile('multirotor.vsp3')

comp_id = vsp.FindGeoms()[2]
# # print(comp_id)
comp_surf = vsp.GetXSecSurf(comp_id, 0)
# xsec_num = vsp.GetNumXSec(comp_surf)
# # print(xsec_num)
# for i in range(1,xsec_num-1):
# 	xsec = vsp.GetXSec(comp_surf, i)
# # 	# print(vsp.GetParmVal(comp_id, 'Dihedral', f'XSec_{i}'))
# 	print(vsp.GetParmVal(vsp.GetXSecParm(xsec, 'Ellipse_Height')))

# print(vsp.GetParmVal(comp_id, 'Rots_Attach_Flag', 'Attach'))


# # # # print(vsp.GetParmVal(wing_id,'Diameter','XSecCurve1'))
# xsec = vsp.GetXSec(comp_surf, 1)
parm_groups 	= vsp.FindContainerGroupNames(comp_id)
print(parm_groups)

parm_ids 	= vsp.FindContainerParmIDs(comp_id) 
for parm_id in parm_ids:
	parm_name = vsp.GetParmName(parm_id)
	print(parm_name)

# print(vsp.GetParmVal(wing_id, 'PropMode', 'Design'))
# print(vsp.GetParmVal(wing_id, 'Diameter', 'Design'))
# print(vsp.GetParmVal(wing_id, 'NumBlade', 'Design'))

# # print(vsp.GetParmVal(wing_id, 'Angle', 'Design'))
# # # print(vsp.GetParmVal(wing_id, 'Chevron_Type', parm_groups[3]))


