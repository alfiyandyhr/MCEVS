import openvsp as vsp
from .Components.Fuselage import NASA_QR_Fuselage, NASA_LPC_Fuselage
from .Components.Wing import NASA_LPC_Wing
from .Components.Tail import NASA_LPC_Horizontal_Tail, NASA_LPC_Vertical_Tail
from .Components.Landing_Gear import NASA_QR_Landing_Gear, NASA_LPC_Landing_Gear
from .Components.Rotor import NASA_QR_Lift_Rotor, NASA_LPC_Lift_Rotor, NASA_LPC_Propeller
from .Components.Payload import Human
from .Components.Boom import NASA_QR_Boom, NASA_LPC_Boom


def create_NASA_QuadRotor_vsp3(fname: str, vehicle: object):

    # Unpacking parameters
    config = vehicle.configuration
    n_pax = vehicle.fuselage.number_of_passenger
    l_fuse = vehicle.fuselage.length
    d_fuse_max = vehicle.fuselage.max_diameter
    n_lift_rotor = vehicle.lift_rotor.n_rotor
    n_blade_rotor = vehicle.lift_rotor.n_blade
    r_lift_rotor = vehicle.lift_rotor.radius
    r_hub = vehicle.lift_rotor.hub_radius
    gear_type = vehicle.landing_gear.gear_type
    l_strut = vehicle.landing_gear.strut_length
    skid_heights = vehicle.landing_gear.skid_heights
    skid_length = vehicle.landing_gear.skid_length

    _ = Human(N_PAX=n_pax, config=config)
    fuse_id = NASA_QR_Fuselage(l_fuse=l_fuse, d_fuse_max=d_fuse_max)
    boom_ids = NASA_QR_Boom(n_lift_rotor=4, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, d_fuse_max=d_fuse_max, fuse_id=fuse_id)
    _ = NASA_QR_Lift_Rotor(n_lift_rotor=n_lift_rotor, r_lift_rotor=r_lift_rotor, r_hub=r_hub, n_blade=n_blade_rotor, l_fuse=l_fuse, d_fuse_max=d_fuse_max, boom_ids=boom_ids)
    _ = NASA_QR_Landing_Gear(gear_type=gear_type, skid_heights=skid_heights, skid_length=skid_length, l_strut=l_strut, fuse_id=fuse_id)
    vsp.WriteVSPFile(fname)
    vsp.ClearVSPModel()


def create_NASA_LiftPlusCruise_vsp3(fname: str, vehicle: object):

    # Unpacking parameters
    config = vehicle.configuration
    n_pax = vehicle.fuselage.number_of_passenger
    l_fuse = vehicle.fuselage.length
    d_fuse_max = vehicle.fuselage.max_diameter
    wing_S = vehicle.wing.area
    wing_AR = vehicle.wing.aspect_ratio
    htail_S = vehicle.horizontal_tail.area
    htail_AR = vehicle.horizontal_tail.aspect_ratio
    vtail_S = vehicle.vertical_tail.area
    vtail_AR = vehicle.vertical_tail.aspect_ratio
    gear_type = vehicle.landing_gear.gear_type
    l_strut = vehicle.landing_gear.strut_length
    n_lift_rotor = vehicle.lift_rotor.n_rotor
    n_blade_rotor = vehicle.lift_rotor.n_blade
    n_propeller = vehicle.propeller.n_propeller
    n_blade_prop = vehicle.propeller.n_blade
    r_lift_rotor = vehicle.lift_rotor.radius
    r_propeller = vehicle.propeller.radius
    l_hub1 = vehicle.lift_rotor.hub_length
    d_hub1 = vehicle.lift_rotor.hub_max_diameter
    l_hub2 = vehicle.propeller.hub_length
    d_hub2 = vehicle.propeller.hub_max_diameter
    l_boom = vehicle.boom.length
    d_boom = vehicle.boom.max_diameter

    Human(N_PAX=n_pax, config=config)
    fuse_id = NASA_LPC_Fuselage(l_fuse, d_fuse_max)
    wing_id = NASA_LPC_Wing(area=wing_S, aspect_ratio=wing_AR, l_fuse=l_fuse, fuse_id=fuse_id)
    _ = NASA_LPC_Horizontal_Tail(area=htail_S, aspect_ratio=htail_AR, l_fuse=l_fuse, fuse_id=fuse_id)
    _ = NASA_LPC_Vertical_Tail(area=vtail_S, aspect_ratio=vtail_AR, l_fuse=l_fuse, fuse_id=fuse_id)
    lg_ids, wheel_ids = NASA_LPC_Landing_Gear(gear_type=gear_type, l_strut=l_strut, fuse_id=fuse_id)
    boom_ids = NASA_LPC_Boom(l_boom=l_boom, d_boom=d_boom, n_lift_rotor=n_lift_rotor, r_lift_rotor=r_lift_rotor, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR, wing_id=wing_id)
    rotor_hub_ids, rotor_ids = NASA_LPC_Lift_Rotor(n_lift_rotor=n_lift_rotor, n_blade=n_blade_rotor, r_lift_rotor=r_lift_rotor, l_hub=l_hub1, d_hub=d_hub1, l_fuse=l_fuse, wing_S=wing_S, wing_AR=wing_AR, boom_ids=boom_ids)
    prop_hub_id, prop_id = NASA_LPC_Propeller(n_propeller=n_propeller, n_blade=n_blade_prop, r_propeller=r_propeller, l_hub=l_hub2, d_hub=d_hub2, l_fuse=l_fuse, fuse_id=fuse_id)
    vsp.WriteVSPFile(fname)
    vsp.ClearVSPModel()
