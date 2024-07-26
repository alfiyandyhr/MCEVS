from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL

design_var = {'wing_area': 8.0, 'wing_AR': 10.0,
			  'r_rotor_lift': 1.0, 'r_rotor_cruise': 1.0,
			  'cruise_speed': 40.0, 'propeller_J': 1.0}

vehicle = StandardLiftPlusCruiseEVTOL(design_var)
# vehicle.print_info()