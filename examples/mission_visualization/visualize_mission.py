import numpy as np
from MCEVS.Missions.Container import Mission
from MCEVS.Missions.Standard import UberMissionProfile
from MCEVS.Missions.Standard import SimplifiedUberMissionProfile
from MCEVS.Missions.Standard import StandardMissionProfile
from MCEVS.Missions.Standard import ConstantHoverAndCruiseMissionProfile
from MCEVS.Utils.Plots import plot_mission_parameters

# Mission range
x = 96600 # m

# Cruise speed
v = 67.1 # m/s

plot_uber = False
plot_simplified_uber = False
plot_standard = False
plot_constantHAC = False

# --- Uber Mission Requirement --- #
if plot_uber:
	mission = UberMissionProfile(v_stall=40)
	plot_mission_parameters(mission, print_info=True)

# --- Simplified Uber Mission Requirement --- #
if plot_simplified_uber:
	mission = SimplifiedUberMissionProfile(mission_range=x, cruise_speed=v)
	plot_mission_parameters(mission, print_info=True)

# --- Standard Mission Requirement --- #
if plot_standard:
	mission = StandardMissionProfile(mission_range=x, cruise_speed=v)
	plot_mission_parameters(mission, print_info=True)

# --- ConstantHoverAndCruise Mission Requirement --- #
if plot_constantHAC:
	mission = ConstantHoverAndCruiseMissionProfile(mission_range=x, cruise_speed=v)
	plot_mission_parameters(mission, print_info=True)