from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Analyses.Aerodynamics.Parasite import calc_flat_plate_area
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
# from MCEVS.Wrappers.OpenVSP.Standard_Vehicles import create_NASA_LiftPlusCruise_vsp3

# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=6500 * 0.3048)
v_inf = 150 * 1609.344 / 3600  # 150 miles/hour = 67.056 m/s # m/s

design_var = {'wing_area': 19.53547845, 'wing_aspect_ratio': 12.12761, 'r_lift_rotor': 1.524, 'r_propeller': 1.3716}
operation_var = {'RPM_lift_rotor': {'hover_climb': 400.0}, 'RPM_propeller': {'cruise': 380.0}}
tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}

vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, tfs, n_pax=6)
# vehicle.print_info()

# create_NASA_LiftPlusCruise_vsp3('liftpluscruise.vsp3', vehicle)

results = calc_flat_plate_area(vehicle, constants['rho'], constants['mu'], v_inf)
f_fuselage = results['f_fuselage']
f_wing = results['f_wing']
f_htail = results['f_horizontal_tail']
f_vtail = results['f_vertical_tail']
f_booms = results['f_booms']
f_LGs = results['f_landing_gears']
f_total_non_hub_non_wing = f_fuselage + f_htail + f_vtail + f_booms + f_LGs
f_total = results['f_total']
Cd0_wing = f_wing / design_var['wing_area']
Cd0_total = f_total / design_var['wing_area']

print(f'f_fuselage = {f_fuselage}')
print(f'f_wing = {f_wing}')
print(f'f_htail = {f_htail}')
print(f'f_vtail = {f_vtail}')
print(f'f_booms = {f_booms}')
print(f'f_landing_gears = {f_LGs}')
print(f'f_total_non_hub_non_wing = {f_total_non_hub_non_wing}')
print(f'f_total = {f_total}')
print(f'Cd0_wing = {Cd0_wing}')
print(f'Cd_total = {Cd0_total}')
print(f"Total drag = {0.5*constants['rho']*v_inf**2*f_total}")
