from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Applications.OpenVSP.Standard_Vehicles import create_NASA_QuadRotor_vsp3, create_NASA_LiftPlusCruise_vsp3

# Cessna 172: AR 7.32, S16.2

design_var1 = {'r_lift_rotor': 1.5, 'rotor_advance_ratio': 0.3}

design_var2 = {'wing_area': 16.2, 'wing_aspect_ratio': 7.32,
			   'r_lift_rotor': 1.0, 'r_propeller': 1.0,
			   'propeller_advance_ratio': 1.0}

vehicle1 = StandardMultirotorEVTOL(design_var1, n_pax=4)
# vehicle1.print_info()

vehicle2 = StandardLiftPlusCruiseEVTOL(design_var2, n_pax=4)
# vehicle2.print_info()

create_NASA_QuadRotor_vsp3('multirotor.vsp3', vehicle1)
create_NASA_LiftPlusCruise_vsp3('liftpluscruise.vsp3', vehicle2)

