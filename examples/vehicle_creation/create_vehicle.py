from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL

design_var1 = {'r_lift_rotor': 1.0}

design_var2 = {'wing_area': 8.0, 'wing_AR': 10.0,
			   'r_rotor_lift': 1.0, 'r_rotor_cruise': 1.0}

vehicle1 = StandardMultirotorEVTOL(design_var1)
vehicle1.print_info()

vehicle2 = StandardLiftPlusCruiseEVTOL(design_var2)
vehicle2.print_info()