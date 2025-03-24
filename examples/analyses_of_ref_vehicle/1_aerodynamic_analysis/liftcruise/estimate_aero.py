from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Analyses.Aerodynamics.Parasite import calc_flat_plate_drag
from MCEVS.Constants.Container import EarthGravityAndAtmosphere

# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=0.0)

# Cessna 172: AR 7.32, S16.2

# design_var1 = {'r_lift_rotor': 1.5, 'rotor_advance_ratio': 0.3}

design_var2 = {'wing_area': 16.2, 'wing_aspect_ratio': 7.32,
			   'r_lift_rotor': 1.0, 'r_propeller': 1.0,
			   'propeller_advance_ratio': 1.0}
# design_var2_opt = {'wing_area':20.0, 'wing_aspect_ratio': 6.0,
# 				   'r_lift_rotor': 1.75, 'r_propeller': 0.87,
# 				   'propeller_advance_ratio': 1.3}

# vehicle1 = StandardMultirotorEVTOL(design_var1, n_pax=4)
# # vehicle1.print_info()

vehicle2 = StandardLiftPlusCruiseEVTOL(design_var2, n_pax=4)
# vehicle2.print_info()
# vehicle2_opt = StandardLiftPlusCruiseEVTOL(design_var2_opt, n_pax=4)

# create_NASA_QuadRotor_vsp3('multirotor.vsp3', vehicle1)
# create_NASA_LiftPlusCruise_vsp3('liftpluscruise.vsp3', vehicle2)
# create_NASA_LiftPlusCruise_vsp3('liftpluscruise_opt.vsp3', vehicle2_opt)

Cd0_total = calc_flat_plate_drag(vehicle2, constants['rho'], constants['mu'], 50.0)/design_var2['wing_area']

print(f'Cd_total = {Cd0_total}')
print(f"Total drag = {0.5*constants['rho']*50**2*Cd0_total*design_var2['wing_area']}")