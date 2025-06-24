import openvsp as vsp
# from MCEVS.Wrappers.OpenVSP.Components.Wing import NASA_LPC_Wing
# from MCEVS.Wrappers.OpenVSP.Components.Fuselage import NASA_QR_Fuselage, NASA_LPC_Fuselage
# from MCEVS.Wrappers.OpenVSP.Components.Boom import NASA_QR_Boom, NASA_LPC_Boom
# from MCEVS.Wrappers.OpenVSP.Components.Rotor import NASA_QR_Lift_Rotor, NASA_LPC_Lift_Rotor, NASA_LPC_Propeller
# from MCEVS.Wrappers.OpenVSP.Components.Landing_Gear import NASA_QR_Landing_Gear

# n_pax = 4

# if n_pax == 1:
# 	l_fuse 		= 2.074	 # m
# 	d_fuse_max 	= 1.4 	 # m
# elif n_pax == 4:
# 	l_fuse 		= 5.2	 # m
# 	d_fuse_max 	= 1.8 	 # m
# print(0.24*1.5)
# fuse_id   = NASA_QR_Fuselage(l_fuse=l_fuse, d_fuse_max=d_fuse_max)
# boom_ids  = NASA_QR_Boom(n_lift_rotor=4, r_lift_rotor=1.5, l_fuse=l_fuse, d_fuse_max=d_fuse_max, fuse_id=fuse_id)
# rotor_ids = NASA_QR_Lift_Rotor(n_lift_rotor=4, r_lift_rotor=1.5, n_blade=5, l_fuse=l_fuse, d_fuse_max=d_fuse_max, boom_ids=boom_ids)
# vsp.WriteVSPFile('vehicle.vsp3')

# fuse_id = NASA_LPC_Fuselage(l_fuse=9.0, d_fuse_max=1.8)
# wing_id = NASA_LPC_Wing(area=16.2, aspect_ratio=7.32, l_fuse=9.0, fuse_id=fuse_id)
# # htail_id = NASA_LPC_Horizontal_Tail(area=htail_S, aspect_ratio=htail_AR, l_fuse=l_fuse, fuse_id=fuse_id)
# # vtail_id = NASA_LPC_Vertical_Tail(area=vtail_S, aspect_ratio=vtail_AR, l_fuse=l_fuse, fuse_id=fuse_id)
# # lg_ids, wheel_ids = NASA_LPC_Landing_Gear(l_strut=l_strut, fuse_id=fuse_id)
# boom_ids = NASA_LPC_Boom(n_lift_rotor=8, r_lift_rotor=1.0, l_fuse=9.0, wing_S=16.2, wing_AR=7.32, wing_id=wing_id)
# rotor_hub_ids, rotor_ids = NASA_LPC_Lift_Rotor(n_lift_rotor=8, n_blade=2, r_lift_rotor=1.0, l_fuse=9.0, wing_S=16.2, wing_AR=7.32, boom_ids=boom_ids)
# prop_hub_id, prop_id = NASA_LPC_Propeller(n_propeller=1, n_blade=5, r_propeller=1.0, l_fuse=9.0, fuse_id=fuse_id)
# vsp.WriteVSPFile('vehicle.vsp3')


vsp.ReadVSPFile('QuadRotor_NASA/rotor.vsp3')

comp_id = vsp.FindGeoms()[0]
# # print(comp_id)
# comp_surf = vsp.GetXSecSurf(comp_id, 0)
# xsec_num = vsp.GetNumXSec(comp_surf)
# print(xsec_num)
# for i in range(1,xsec_num-1):
# 	xsec = vsp.GetXSec(comp_surf, i)
# # 	# print(vsp.GetParmVal(comp_id, 'Dihedral', f'XSec_{i}'))
# 	print(vsp.GetParmVal(vsp.GetXSecParm(xsec, 'Ellipse_Height')))

# print(vsp.GetParmVal(comp_id, 'Rots_Attach_Flag', 'Attach'))


# # # # # # print(vsp.GetParmVal(wing_id,'Diameter','XSecCurve1'))
# # # xsec = vsp.GetXSec(comp_surf, 1)
parm_groups = vsp.FindContainerGroupNames(comp_id)
print(parm_groups)

parm_ids = vsp.FindContainerParmIDs(comp_id)
for parm_id in parm_ids:
    parm_name = vsp.GetParmName(parm_id)
    print(parm_name)

# print(vsp.GetParmVal(comp_id, 'CrvType', 'Chord'))
# print(vsp.GetParmVal(comp_id, 'crd_0', 'Chord'))
# print(vsp.GetParmVal(comp_id, 'UseBeta34Flag', 'Design'))
# print(vsp.GetParmVal(comp_id, 'Beta34', 'Design'))
# print(vsp.GetParmVal(comp_id, 'tw_0', 'Twist'))
# print(vsp.GetParmVal(comp_id, 'RadiusFrac', 'XSec_0'))

# # print(vsp.GetParmVal(wing_id, 'Angle', 'Design'))
# # # print(vsp.GetParmVal(wing_id, 'Chevron_Type', parm_groups[3]))
