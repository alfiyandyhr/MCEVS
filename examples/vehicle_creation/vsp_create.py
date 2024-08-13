import openvsp as vsp
from MCEVS.Applications.OpenVSP.Components.Fuselage import NASA_LPC_Fuselage
from MCEVS.Applications.OpenVSP.Components.Wing import NASA_LPC_Wing
from MCEVS.Applications.OpenVSP.Components.Tail import NASA_LPC_Horizontal_Tail, NASA_LPC_Vertical_Tail
from MCEVS.Applications.OpenVSP.Components.Landing_Gear import NASA_LPC_Landing_Gear
from MCEVS.Applications.OpenVSP.Components.Rotor import NASA_LPC_Lift_Rotor, NASA_LPC_Propeller
from MCEVS.Applications.OpenVSP.Components.Payload import Human
from MCEVS.Applications.OpenVSP.Components.Boom import NASA_LPC_Boom

n_pax = 4

Human(N_PAX=n_pax)

if n_pax == 4:
	l_fuse 		= 9.0	 # m
	d_fuse_max 	= 1.8 	 # m

NASA_LPC_Fuselage(l_fuse, d_fuse_max)
# Cessna 172: AR 7.32, S16.2
NASA_LPC_Wing(area=16.2, aspect_ratio=7.32, l_fuse=l_fuse)
NASA_LPC_Horizontal_Tail(area=2.0, aspect_ratio=2.0, l_fuse=l_fuse)
NASA_LPC_Vertical_Tail(area=2.5, aspect_ratio=1.2, l_fuse=l_fuse)
NASA_LPC_Landing_Gear(l_strut=0.3, l_fuse=l_fuse, d_fuse_max=d_fuse_max)
NASA_LPC_Lift_Rotor(n_lift_rotor=8, n_blade=2, r_lift_rotor=1.0, l_fuse=l_fuse, wing_S=16.2, wing_AR=7.32)
NASA_LPC_Propeller(n_propeller=1, n_blade=4, r_propeller=1.0, l_fuse=l_fuse)
NASA_LPC_Boom(n_lift_rotor=8, l_fuse=l_fuse, wing_S=16.2, wing_AR=7.32)
vsp.WriteVSPFile('vehicle.vsp3')

# vsp.ReadVSPFile('NASA_LPC/prop_example.vsp3')

# wing_id = vsp.FindGeoms()[0]
# # # wing_surf = vsp.GetXSecSurf(wing_id, 0)
# # # xsec_num = vsp.GetNumXSec(wing_surf)
# # # # # # print(xsec_num)
# # # for i in range(1,xsec_num-1):
# # # 	xsec = vsp.GetXSec(wing_surf, i)
# # # 	print(vsp.GetParmVal(vsp.GetXSecParm(xsec, 'Circle_Diameter')))
# # # 	vsp.GetXSecParm(xsec, 'Height')
# # # for i in range(1,4):
# # 	# print(vsp.GetParmVal(wing_id, 'Circle_Diameter', f'XSec_{i}'))
# # # # 	# print(vsp.GetParmVal(wing_id, 'SectTess_U', f'XSec_{i}'))
# # # # 	# print(vsp.GetParmVal(wing_id, 'InLEMode', f'XSec_{i}'))
# # # # 	# print(vsp.GetParmVal(wing_id, 'InLEStrength', f'XSec_{i}'))
# # # # 	# print(vsp.GetParmVal(wing_id, 'InLESweep', f'XSec_{i}'))
# # # # 	# print(vsp.GetParmVal(wing_id, 'InLEDihedral', f'XSec_{i}'))



# # # # print(vsp.GetParmVal(wing_id,'Diameter','XSecCurve1'))
# # # xsec = vsp.GetXSec(wing_surf, 1)
# parm_groups 	= vsp.FindContainerGroupNames(wing_id)
# print(parm_groups)

# parm_ids 	= vsp.FindContainerParmIDs(wing_id) 
# for parm_id in parm_ids:
# 	parm_name = vsp.GetParmName(parm_id)
# 	print(parm_name)

# print(vsp.GetParmVal(wing_id, 'PropMode', 'Design'))
# print(vsp.GetParmVal(wing_id, 'Diameter', 'Design'))
# print(vsp.GetParmVal(wing_id, 'NumBlade', 'Design'))

# # print(vsp.GetParmVal(wing_id, 'Angle', 'Design'))
# # # print(vsp.GetParmVal(wing_id, 'Chevron_Type', parm_groups[3]))




# # # # # xsec = vsp.GetXSec(wing_surf, 0)
# # # parm_ids = vsp.FindContainerParmIDs( wing_id )

# # # print(parm_ids)
# # # for i in range(len(parm_ids)):
# # #     name_id = vsp.GetParmName( parm_ids[i] ) + ": " + parm_ids[i] + "\n"
# # #     print( name_id )



