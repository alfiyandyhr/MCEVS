from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Missions.Container import Mission
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
from MCEVS.Analyses.Weight.Analysis import WeightAnalysis
from MCEVS.Optimization.Container import DesignProblem
from MCEVS.Utils.Plots import plot_mission_parameters

# Mission requirement
mission_range = 30000 # m
cruise_speed = 30.0 # m/s
payload_weight = 400.0

# Take-off from 5000 ft ASL or 1500 m
mission = Mission(takeoff_altitude=0.0)
# mission.add_segment('Hover Climb', kind='HoverStay', duration=120.0, n_discrete=5)
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=2.54, distance=304.8, n_discrete=10)
mission.add_segment('Cruise', kind='CruiseConstantSpeed', AoA=5, speed=cruise_speed, distance=mission_range, n_discrete=5)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=1.524, distance=304.8, n_discrete=10)
# mission.add_segment('Hover Descent', kind='HoverStay', duration=120.0, n_discrete=5)
# plot_mission_parameters(mission, print_info=False)

# Constants
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=mission.takeoff_altitude)

# Initial design variables (aircraft design and operation)
design_var = {'r_lift_rotor': 1.0, 'cruise_speed': cruise_speed, 'rotor_advance_ratio': 0.3}

vehicle = StandardMultirotorEVTOL(design_var)

# # Analysis
# analysis = WeightAnalysis(vehicle=vehicle, mission=mission, constants=constants, sizing_mode=True)

# Design problem
problem = DesignProblem(vehicle=vehicle,
						mission=mission,
						constants=constants,
						algorithm='gradient-based')

problem.add_objective('Weight|takeoff')
problem.add_design_var('Mission|cruise_speed', 20.0, 60.0, 'm/s')
problem.add_design_var('LiftRotor|radius', 0.5, 1.5, 'm')
problem.add_design_var('LiftRotor|advance_ratio', 0.01, 0.5)
problem.add_constraint('LiftRotor|thrust_coefficient', 0.0, 0.14*vehicle.lift_rotor.solidity)
problem.add_constraint('DiskLoading|LiftRotor|segment_1', 0.0, 600.0, 'N/m**2')
problem.add_constraint('DiskLoading|LiftRotor|segment_2', 0.0, 600.0, 'N/m**2')
problem.add_constraint('DiskLoading|LiftRotor|segment_3', 0.0, 600.0, 'N/m**2')

# Optimization
res = problem.run_optimization()

# Optimal design var
print('v_cruise =', res.get_val('Mission|cruise_speed', 'm/s'), 'm/s')
print('mu_rotor =', res.get_val('LiftRotor|advance_ratio'))
print('r_rotor =', res.get_val('LiftRotor|radius', 'm'), 'm')
print('MTOW =', res.get_val('Weight|takeoff', 'kg'), 'kg')








