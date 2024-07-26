from MCEVS.Missions.Standard import ConstantHoverAndCruiseMissionProfile
from MCEVS.Utils.Plots import plot_mission_parameters

# Mission range
x = 96600 # m

# Cruise speed
v = 67.1 # m/s

mission = ConstantHoverAndCruiseMissionProfile(mission_range=x, cruise_speed=v)
# plot_mission_parameters(mission, print_info=True)