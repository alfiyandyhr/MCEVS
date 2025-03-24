from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Analyses.Aerodynamics.Parasite import calc_flat_plate_drag
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
import numpy as np

# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=0.0)

# Cessna 172: AR 7.32, S16.2

design_var = {'r_lift_rotor': 1.5, 'rotor_advance_ratio': 0.3}

vehicle = StandardMultirotorEVTOL(design_var, n_pax=4)
# vehicle.print_info()

# create_NASA_QuadRotor_vsp3('multirotor.vsp3', vehicle)
Cd0_total = calc_flat_plate_drag(vehicle, constants['rho'], constants['mu'], 50.0)/(np.pi*design_var['r_lift_rotor']**2)

print(f'Cd_total = {Cd0_total}')
print(f"Total drag = {0.5*constants['rho']*50**2*Cd0_total*np.pi*design_var['r_lift_rotor']**2}")