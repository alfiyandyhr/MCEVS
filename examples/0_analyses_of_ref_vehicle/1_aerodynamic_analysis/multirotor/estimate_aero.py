from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Analyses.Aerodynamics.Parasite import calc_flat_plate_area
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
# from MCEVS.Wrappers.OpenVSP.Standard_Vehicles import create_NASA_QuadRotor_vsp3
import numpy as np

# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=6500 * 0.3048)
v_inf = 150 * 1609.344 / 3600  # 150 miles/hour = 67.056 m/s # m/s

design_var = {'r_lift_rotor': 4.20624}  # 13.8 ft = 4.20624 m
operation_var = {'RPM_lift_rotor': {'hover_climb': None, 'cruise': 450.0}}
tfs = {'tf_structure': 0.8, 'tf_propulsion': 0.8, 'tf_equipment': 0.8}

vehicle = StandardMultirotorEVTOL(design_var, operation_var, tfs, n_pax=6)
# vehicle.print_info()

# create_NASA_QuadRotor_vsp3('multirotor.vsp3', vehicle)

results = calc_flat_plate_area(vehicle, constants['rho'], constants['mu'], v_inf)
f_total = results['f_total']
f_fuselage = results['f_fuselage']
f_booms = results['f_booms']
f_LGs = results['f_landing_gears']
Cd0_total = f_total / (np.pi * design_var['r_lift_rotor']**2)

print(f'f_fuselage = {f_fuselage}')
print(f'f_booms = {f_booms}')
print(f'f_landing_gears = {f_LGs}')
print(f'f_total = {f_total}')
print(f'Cd_total = {Cd0_total}')
print(f"Total drag = {0.5*constants['rho']*v_inf**2*f_total}")
