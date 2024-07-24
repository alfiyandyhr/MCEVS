import numpy as np
from MCEVS.Missions.Standard import UberMissionRequirement
from MCEVS.Missions.Standard import SimpleMissionRequirement


# mission = UberMissionRequirement(v_stall=40)
# # mission.print()
# mission.visualize()
# 96600
mission = SimpleMissionRequirement(mission_range=96600, cruise_speed=67.1)
# mission.print()
mission.visualize()