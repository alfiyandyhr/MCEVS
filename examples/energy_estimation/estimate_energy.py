from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Standard import ConstantHoverAndCruiseMissionProfile
from MCEVS.Analyses.Power import PowerAnalysis
from MCEVS.Utils.Plots import plot_mission_parameters

# Mission requirement
mission_range = 30000 # m
cruise_speed = 30.0 # m/s

mission = ConstantHoverAndCruiseMissionProfile(mission_range=mission_range, cruise_speed=cruise_speed)
# plot_mission_parameters(mission, print_info=False)

# Design variables (aircraft design and operation)
# design_var1 = {'r_lift_rotor': 1.0, 'cruise_speed': cruise_speed, 'lift_rotor_mu': 0.3}
design_var2 = {'wing_area': 8.0, 'wing_AR': 10.0,
			  'r_lift_rotor': 1.0, 'r_propeller': 1.0,
			  'cruise_speed': cruise_speed, 'propeller_J': 1.0}

# vehicle1 = StandardMultirotorEVTOL(design_var1)
# vehicle1.weight.max_takeoff = 1500.0

vehicle2 = StandardLiftPlusCruiseEVTOL(design_var2)
vehicle2.weight.max_takeoff = 1500.0

analysis = PowerAnalysis(vehicle2, mission)
results = analysis.evaluate()
