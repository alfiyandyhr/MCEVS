import numpy as np
from MCEVS.Missions.Container import Mission

mission = Mission()
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=2.5, distance=150.0)
# mission.add_segment(name='Cruise', kind='CruiseConstantSpeed', speed=26.8, distance=96500.0)
# mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=1.5, distance=150.0)
mission.print()
mission.visualize()
# print(mission.ay)

