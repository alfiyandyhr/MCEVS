from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
from MCEVS.Analyses.Power.Analysis import PowerAnalysis
from MCEVS.Utils.Plots import plot_mission_parameters

# Mission requirement
mission_range = 30000 # m
cruise_speed = 30.0 # m/s
payload_weight = 400.0

# Take-off from 5000 ft ASL or 1500 m
mission = Mission(takeoff_altitude=0.0)
# mission.add_segment('Hover Climb', kind='HoverStay', duration=120.0, n_discrete=5)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=2.54, distance=152.4, n_discrete=10)
mission.add_segment('Cruise', kind='CruiseConstantSpeed', AoA=5, speed=cruise_speed, distance=mission_range, n_discrete=5)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=1.524, distance=152.4, n_discrete=10)
# mission.add_segment('Hover Descent', kind='HoverStay', duration=120.0, n_discrete=5)
# plot_mission_parameters(mission, print_info=False)

# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=mission.takeoff_altitude)

# Design variables (aircraft design and operation)
design_var1 = {'r_lift_rotor': 1.0, 'cruise_speed': cruise_speed, 'rotor_advance_ratio': 0.3}
design_var2 = {'wing_area': 8.0, 'wing_aspect_ratio': 10.0,
			  'r_lift_rotor': 1.0, 'r_propeller': 1.0,
			  'cruise_speed': cruise_speed, 'propeller_advance_ratio': 1.0}

vehicle1 = StandardMultirotorEVTOL(design_var1, mtow=878.439221)
vehicle2 = StandardLiftPlusCruiseEVTOL(design_var2, mtow=945.145355)

# Analysis
analysis = PowerAnalysis(vehicle=vehicle1, mission=mission, constants=constants)
results = analysis.evaluate()

# print(results.get_val('Power|LiftRotor|segment_1'))
# print(results.get_val('Power|LiftRotor|segment_2'))
# print(results.get_val('Power|LiftRotor|segment_3'))
# print(results.get_val('Power|LiftRotor|maximum'))
# print(results.get_val('Power|Propeller|segment_1'))
# print(results.get_val('Power|Propeller|segment_2'))
# print(results.get_val('Power|Propeller|segment_3'))
# print(results.get_val('Power|Propeller|maximum'))
# print(results.get_val('Power|segment_1'))
# print(results.get_val('Power|segment_2'))
# print(results.get_val('Power|segment_3'))

print(results.get_val('LiftRotor|thrust_each|segment_1'))
print(results.get_val('LiftRotor|thrust_each|segment_2'))
print(results.get_val('LiftRotor|thrust_each|segment_3'))
print(results.get_val('Propeller|thrust_each|segment_1'))
print(results.get_val('Propeller|thrust_each|segment_2'))
print(results.get_val('Propeller|thrust_each|segment_3'))

print(results.get_val('DiskLoading|LiftRotor|segment_1'))
print(results.get_val('DiskLoading|LiftRotor|segment_2'))
print(results.get_val('DiskLoading|LiftRotor|segment_3'))
print(results.get_val('DiskLoading|Propeller|segment_1'))
print(results.get_val('DiskLoading|Propeller|segment_2'))
print(results.get_val('DiskLoading|Propeller|segment_3'))







