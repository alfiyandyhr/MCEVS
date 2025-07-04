import openvsp as vsp
from .Components.Fuselage import NASA_QR_Fuselage, NASA_LPC_Fuselage
from .Components.Wing import NASA_LPC_Wing
from .Components.Tail import NASA_LPC_Horizontal_Tail, NASA_LPC_Vertical_Tail
from .Components.Landing_Gear import NASA_QR_Landing_Gear, NASA_LPC_Landing_Gear
# from .Components.Rotor import NASA_QR_Lift_Rotor, NASA_LPC_Lift_Rotor, NASA_LPC_Propeller
from .Components.Boom import NASA_QR_Boom, NASA_LPC_Boom


def calc_wetted_area(vehicle: object):

    # Unpacking parameters
    config = vehicle.configuration
    # n_pax = vehicle.fuselage.number_of_passenger
    l_fuse = vehicle.fuselage.length
    d_fuse_max = vehicle.fuselage.max_diameter
    gear_type = vehicle.landing_gear.gear_type
    l_strut = vehicle.landing_gear.strut_length
    skid_heights = vehicle.landing_gear.skid_heights
    skid_length = vehicle.landing_gear.skid_length
    n_lift_rotor = vehicle.lift_rotor.n_rotor
    # n_blade_rotor = vehicle.lift_rotor.n_blade
    r_lift_rotor = vehicle.lift_rotor.radius

    if config == 'LiftPlusCruise':
        wing_S = vehicle.wing.area
        wing_AR = vehicle.wing.aspect_ratio
        htail_S = vehicle.horizontal_tail.area
        htail_AR = vehicle.horizontal_tail.aspect_ratio
        vtail_S = vehicle.vertical_tail.area
        vtail_AR = vehicle.vertical_tail.aspect_ratio
        # n_propeller = vehicle.propeller.n_propeller
        # n_blade_prop = vehicle.propeller.n_blade
        # r_propeller = vehicle.propeller.radius
        l_boom = vehicle.boom.length
        d_boom = vehicle.boom.max_diameter

    if config == 'Multirotor':
        fuse_id = NASA_QR_Fuselage(l_fuse, d_fuse_max)
        boom_ids = NASA_QR_Boom(n_lift_rotor=n_lift_rotor, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, d_fuse_max=d_fuse_max, fuse_id=fuse_id)
        lg_ids = NASA_QR_Landing_Gear(gear_type=gear_type, skid_heights=skid_heights, skid_length=skid_length, l_strut=l_strut, fuse_id=fuse_id)
        # Forwarding area and aspect ratio of booms
        if vehicle.boom.area is None:
            vehicle.boom.area = []
            for boom_id in boom_ids:
                vehicle.boom.area.append(vsp.GetParmVal(boom_id, 'TotalArea', 'WingGeom'))
        if vehicle.boom.aspect_ratio is None:
            vehicle.boom.aspect_ratio = []
            for boom_id in boom_ids:
                vehicle.boom.aspect_ratio.append(vsp.GetParmVal(boom_id, 'TotalAR', 'WingGeom'))
        # vsp.WriteVSPFile('multirotor_check.vsp3')

        _ = vsp.ComputeCompGeom(vsp.SET_ALL, False, 0)
        comp_res_id = vsp.FindLatestResultsID('Comp_Geom')
        double_arr = vsp.GetDoubleResults(comp_res_id, 'Wet_Area')

        res = {}
        res['Fuselage'] = double_arr[0]
        res['Boom_1'] = double_arr[1]
        res['Boom_2'] = double_arr[2]
        res['Boom_3'] = double_arr[3]
        res['Boom_4'] = double_arr[4]
        if gear_type == 'wheeled':
            res['NoseStrut_LG'] = double_arr[5]
            res['NoseWheel_LG'] = double_arr[6]
            res['MainStrut_LG_1'] = double_arr[7]
            res['MainWheel_LG_1'] = double_arr[8]
            res['MainStrut_LG_2'] = double_arr[9]
            res['MainWheel_LG_2'] = double_arr[10]
        elif gear_type == 'skid':
            res['FrontStrut_1'] = double_arr[5]
            res['RightSkid'] = double_arr[6]
            res['FrontStrut_2'] = double_arr[7]
            res['LeftSkid'] = double_arr[8]
            res['RearStrut_1'] = double_arr[9]
            res['RearStrut_2'] = double_arr[10]

        vsp.ClearVSPModel()

        return res

    elif config == 'LiftPlusCruise':
        fuse_id = NASA_LPC_Fuselage(l_fuse, d_fuse_max)
        wing_id = NASA_LPC_Wing(area=wing_S, aspect_ratio=wing_AR, l_fuse=l_fuse, fuse_id=fuse_id)
        _ = NASA_LPC_Horizontal_Tail(area=htail_S, aspect_ratio=htail_AR, l_fuse=l_fuse, fuse_id=fuse_id)
        _ = NASA_LPC_Vertical_Tail(area=vtail_S, aspect_ratio=vtail_AR, l_fuse=l_fuse, fuse_id=fuse_id)
        lg_ids, wheel_ids = NASA_LPC_Landing_Gear(gear_type=gear_type, l_strut=l_strut, fuse_id=fuse_id)
        boom_ids = NASA_LPC_Boom(l_boom=l_boom, d_boom=d_boom, n_lift_rotor=n_lift_rotor, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR, wing_id=wing_id)
        # vsp.WriteVSPFile('liftpluscruise_check.vsp3')

        _ = vsp.ComputeCompGeom(vsp.SET_ALL, False, 0)
        comp_res_id = vsp.FindLatestResultsID('Comp_Geom')
        double_arr = vsp.GetDoubleResults(comp_res_id, 'Wet_Area')

        res = {}
        res['Fuselage'] = double_arr[0]
        res['Wing'] = double_arr[1]
        res['HTail'] = double_arr[6]
        res['VTail'] = double_arr[7]
        res['Boom_1'] = double_arr[2]
        res['Boom_2'] = double_arr[3]
        res['Boom_3'] = double_arr[4]
        res['Boom_4'] = double_arr[5]
        if gear_type == 'wheeled':
            res['NoseStrut_LG'] = double_arr[8]
            res['NoseWheel_LG'] = double_arr[9]
            res['MainStrut_LG_1'] = double_arr[10]
            res['MainWheel_LG_1'] = double_arr[11]
            res['MainStrut_LG_2'] = double_arr[12]
            res['MainWheel_LG_2'] = double_arr[13]
        elif gear_type == 'skid':
            res['FrontStrut_1'] = double_arr[8]
            res['RightSkid'] = double_arr[9]
            res['FrontStrut_2'] = double_arr[10]
            res['LeftSkid'] = double_arr[11]
            res['RearStrut_1'] = double_arr[12]
            res['RearStrut_2'] = double_arr[13]

        vsp.ClearVSPModel()

        return res
